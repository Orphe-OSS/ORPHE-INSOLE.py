import asyncio
import struct
from datetime import datetime, timedelta
from bleak import BleakClient, BleakScanner


# キャラクタリスティックUUID
CHARACTERISTIC_SENSOR_VALUES_UUID = "f3f9c7ce-46ee-4205-89ac-abe64e626c0f"
CHARACTERISTIC_STEP_ANALYSIS_UUID = "4eb776dc-cf99-4af7-b2d3-ad0f791a79dd"
CHARACTERISTIC_DEVICE_INFORMATION_UUID = "24354f22-1c46-430e-a4ab-a1eeabbcdfc0"

# デバイスのサービスUUID（例：OrpheのサービスUUID）
SERVICE_ORPHE_INFORMATION_UUID = "01a9d6b5-ff6e-444a-b266-0be75e85c064"
SERVICE_OTHER_UUID = "db1b7aca-cda5-4453-a49b-33a53d3f0833"

WRITE_WAIT_INTERVAL_SEC = 0.5


def to_timestamp(hours, minutes, seconds, ms_high, ms_low):
    """
    v3のデータについてくるタイムスタンプをフォーマットする関数

    Args:
        hours: 時
        minutes: 分
        seconds: 秒
        ms_high: ミリ秒の上位バイト
        ms_low: ミリ秒の下位バイト
    """
    # 現在の日付を取得
    now = datetime.now()

    # ミリ秒を上位バイトと下位バイトから計算
    milliseconds = (ms_high << 8) | ms_low

    # 日付を置き換えて、新しいdatetimeオブジェクトを作成
    dt = now.replace(hour=hours, minute=minutes, second=seconds,
                     microsecond=milliseconds * 1000)

    # UNIXタイムスタンプを返す（MSまで含めた整数）
    return int(dt.timestamp()*1000)


class AccData:
    """
    加速度センサの値を格納するクラス

    Attributes:
        x: x軸の加速度
        y: y軸の加速度
        z: z軸の加速度
        timestamp: タイムスタンプ
        serial_number: シリアルナンバー
        packet_number: パケットナンバー
    """

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.timestamp = 0
        self.serial_number = 0
        self.packet_number = 0

    def print(self):
        print(
            f"Acc[{self.serial_number}][{self.packet_number}][{self.timestamp}]: {self.x}, {self.y}, {self.z}")


class GyroData:
    """
    ジャイロセンサの値を格納するクラス

    Attributes:
        x: x軸の角速度
        y: y軸の角速度
        z: z軸の角速度
        timestamp: タイムスタンプ
        serial_number: シリアルナンバー
        packet_number: パケットナンバー
    """

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.timestamp = 0
        self.serial_number = 0
        self.packet_number = 0

    def print(self):
        print(
            f"Gyro[{self.serial_number}][{self.packet_number}][{self.timestamp}]: {self.x}, {self.y}, {self.z}")


class QuatData:
    """
    クォータニオンの値を格納するクラス

    Attributes:
        w: w
        x: x
        y: y
        z: z
        timestamp: タイムスタンプ
        serial_number: シリアルナンバー
        packet_number: パケットナンバー
    """

    def __init__(self):
        self.w = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.timestamp = 0
        self.serial_number = 0
        self.packet_number = 0

    def print(self):
        print(
            f"Quat[{self.serial_number}][{self.packet_number}][{self.timestamp}]: {self.w}, {self.x}, {self.y}, {self.z}")


class PressureData:
    """
    圧力センサの値を格納するクラス

    Attributes:
        values[6]: センサの圧力値
        timestamp: タイムスタンプ
        serial_number: シリアルナンバー
        packet_number: パケットナンバー
    """

    def __init__(self):
        self.values = [6]*0
        self.timestamp = 0
        self.serial_number = 0
        self.packet_number = 0

    def print(self):
        print(
            f"Pressure[{self.serial_number}][{self.packet_number}][{self.timestamp}]: {self.values}")


class Range:
    """
    加速度センサとジャイロセンサのレンジを格納するクラス

    Attributes:
        acc: 加速度センサのレンジ
        gyro: ジャイロセンサのレンジ
    """

    def __init__(self):
        self.acc = 0
        self.gyro = 0


class GaitData:
    """
    歩行解析の値を格納するクラス

    Attributes:
        step_count: 歩数
        gait_type: 歩容タイプ（0:無し、1:歩行、2:走行,3:直立静止）
        direction: ストライド方向（0:なし, 1:前方, 2:後方,3:内側,4:外側）
        calorie: 総消費カロリー
        distance: 総移動距離
        standing_phase_duration: 立脚期継続時間
        swing_phase_duration: 遊脚期継続時間
    """

    def __init__(self, data):
        # 2,3は Uint16 で歩数が入っている
        self.step_count = int.from_bytes(
            data[2:4], byteorder='big', signed=False)
        # 4番目は最初の2ビット分が enumで歩容タイプ（0:無し、1:歩行、2:走行,3:直立静止）
        self.gait_type = data[4] & 0b11000000 >> 6
        # 4番目は2,3,4ビット分がenumでストライド方向（0:なし, 1:前方, 2:後方,3:内側,4:外側）
        self.direction = (data[4] & 0b00111000) >> 3
        # data[6],data[7]はfloat16で総消費カロリー
        self.calorie = struct.unpack('>e', data[6:8])
        # 8,9,10,11はfloat32で総移動距離
        self.distance = struct.unpack('>f', data[8:12])
        # 12,13,14,15はfloat32で立脚期継続時間（standing phase duration）
        self.standing_phase_duration = struct.unpack('>f', data[12:16])
        # 16,17,18,19はflaot32で遊脚期継続時間（swing_phase_duration)
        self.swing_phase_duration = struct.unpack('>f', data[16:20])

    def print(self):
        print(f"Step count: {self.step_count}")
        print(f"Gait type: {self.gait_type}")
        print(f"Direction: {self.direction}")
        print(f"Calorie: {self.calorie}")
        print(f"Distance: {self.distance}")
        print(f"Standing phase duration: {self.standing_phase_duration}")
        print(f"Swing phase duration: {self.swing_phase_duration}")


class StrideData:
    """
    ストライドの値を格納するクラス

    Attributes:
        step_count: 歩数
        foot_angle: フットアングル
        x: ストライドX
        y: ストライドY
        z: ストライドZ
    """

    def __init__(self, data):
        # 2,3は Uint16 で歩数が入っている
        self.step_count = int.from_bytes(
            data[2:4], byteorder='big', signed=False)
        # 4,5,6,7はfloat32でフットアングル
        self.foot_angle = struct.unpack('>f', data[4:8])
        # 8,9,10,11はfloat32でストライドX
        self.x = struct.unpack('>f', data[8:12])
        # 12,13,14,15はfloat32でストライドY
        self.y = struct.unpack('>f', data[12:16])
        # 16,17,18,19はfloat32でストライドZ
        self.z = struct.unpack('>f', data[16:20])

    def print(self):
        print(f"Step count: {self.step_count}")
        print(f"Foot angle: {self.foot_angle}")
        print(f"Stride X: {self.x}")
        print(f"Stride Y: {self.y}")
        print(f"Stride Z: {self.z}")


class PronationData:
    """
    プロネーションの値を格納するクラス

    Attributes:
        step_count: 歩数
        landing_impact: 着地衝撃力
        x: プロネーションX
        y: プロネーションY
        z: プロネーションZ
    """

    def __init__(self, data):
        # 2,3は Uint16 で歩数が入っている
        self.step_count = int.from_bytes(
            data[2:4], byteorder='big', signed=False)
        # 4,5,6,7はfloat32で着地衝撃力[kgf](landing_impact)
        self.landing_impact = struct.unpack('>f', data[4:8])
        # 8,9,10,11はプロネーションX[deg](x)
        self.x = struct.unpack('>f', data[8:12])
        # 12,13,14,15はプロネーションY[deg](y)
        self.y = struct.unpack('>f', data[12:16])
        # 16,17,18,19はプロネーションZ[deg](z)
        self.z = struct.unpack('>f', data[16:20])

    def print(self):
        print(f"Step count: {self.step_count}")
        print(f"Landing impact: {self.landing_impact}")
        print(f"Pronation X: {self.x}")
        print(f"Pronation Y: {self.y}")
        print(f"Pronation Z: {self.z}")


class QuatDistanceData:
    """
    クォータニオンと差分値を格納するクラス

    Attributes:
        step_count: 歩数
        phase: 歩容フェイズ
        period: 歩容ピリオド
        event: 歩容イベント
        w: クォータニオンのw
        x: クォータニオンのx
        y: クォータニオンのy
        z: クォータニオンのz
        x_distance: 加速度力算出された単位時間のx移動距離
        y_distance: 加速度力算出された単位時間のy移動距離
        z_distance: 加速度力算出された単位時間のz移動距離
    """

    def __init__(self, data):
        # 2,3は Uint16 で歩数が入っている
        self.step_count = int.from_bytes(
            data[2:4], byteorder='big', signed=False)
        # 4は01ビットがenumの歩容フェイズ（0:なし, 1:立脚期, 2:遊脚期）
        self.phase = data[4] & 0b00000001
        # 4は2,3,4ビットがenumの歩容ピリオド（0:なし,1:LoadingResponse, 2:MidStance, 3:TerminalStance, 4:InitialSwing, 5:MidSwing, 6:TerminalSwing）
        self.period = (data[4] & 0b00011110) >> 1
        # 4は5,6,7ビットがenumの歩容イベント(0:なし, 1:InitialContact, 2:FootFlat, 3:HeelRise, 4:ToeOff, 5:FeetAdjacent, 6:TibiaVertical)
        self.event = (data[4] & 0b11100000) >> 5
        # 6,7はfloat16でクォータニオンのw
        self.w = struct.unpack('>e', data[6:8])
        # 8,9はfloat16でクォータニオンのx
        self.x = struct.unpack('>e', data[8:10])
        # 10,11はfloat16でクォータニオンのy
        self.y = struct.unpack('>e', data[10:12])
        # 12,13はfloat16でクォータニオンのz
        self.z = struct.unpack('>e', data[12:14])
        # 14,15はfloat16で加速度力算出された単位時間のx移動距離
        self.x_distance = struct.unpack('>e', data[14:16])
        # 16,17はfloat16で加速度力算出された単位時間のy移動距離
        self.y_distance = struct.unpack('>e', data[16:18])
        # 18,19はfloat16で加速度力算出された単位時間のz移動距離
        self.z_distance = struct.unpack('>e', data[18:20])

    def print(self):
        print(f"Step count: {self.step_count}")
        print(f"Phase: {self.phase}")
        print(f"Period: {self.period}")
        print(f"Event: {self.event}")
        print(f"Quat: {self.w}, {self.x}, {self.y}, {self.z}")
        print(
            f"Distance: {self.x_distance}, {self.y_distance}, {self.z_distance}")


class StepAnalysisData:
    """
    ステップ解析の値を格納するクラス
    """

    def __init__(self, owner, data):
        """
        コンストラクタ

        data[1]のサブヘッダによって解析データの種類がわかる。対応するのは、0,1,2,3,4 である。また、0,1,2,3 に関してはデータ到着担保のために同じデータが二回連続で送信されてくるため、歩数カウントで更新すべきデータかどうかを判断する必要がある。

        0: Gait Overview
        1: Stride
        2: Pronation
        3: 未実装
        4: クオータニオン
        5: 未実装
        6: 未実装

        Args:
            owner: オーナー（StepAnalysisDataを生成したオブジェクトで、Orpheクラスのインスタンスが入っている。これはコールバック関数を登録するため）
            data: 生データ
        """
        self.data = data

        # gait overview
        if data[1] == 0:
            self.step_count = int.from_bytes(
                data[2:4], byteorder='big', signed=False)
            if self.step_count > owner.step_count.gait:
                self.gait = GaitData(data)
                # コールバック関数が設定されている場合、コールバック関数を呼び出す
                if hasattr(owner, 'got_gait_callback') and owner.got_gait_callback:
                    owner.got_gait_callback(self.gait)
            owner.step_count.gait = self.step_count
        # stride
        elif data[1] == 1:
            self.step_count = int.from_bytes(
                data[2:4], byteorder='big', signed=False)
            if self.step_count > owner.step_count.stride:
                self.stride = StrideData(data)
                # コールバック関数が設定されている場合、コールバック関数を呼び出す
                if hasattr(owner, 'got_stride_callback') and owner.got_stride_callback:
                    owner.got_stride_callback(self.stride)
            owner.step_count.stride = self.step_count
        # pronation
        elif data[1] == 2:
            self.step_count = int.from_bytes(
                data[2:4], byteorder='big', signed=False)
            if self.step_count > owner.step_count.pronation:
                self.pronation = PronationData(data)
                # コールバック関数が設定されている場合、コールバック関数を呼び出す
                if hasattr(owner, 'got_pronation_callback') and owner.got_pronation_callback:
                    owner.got_pronation_callback(self.pronation)
            owner.step_count.pronation = self.step_count
        # quaternion
        elif data[1] == 4:
            self.step_count = int.from_bytes(
                data[2:4], byteorder='big', signed=False)
            # if self.step_count > owner.step_count.quat:
            self.quat_distance = QuatDistanceData(data)
            # コールバック関数が設定されている場合、コールバック関数を呼び出す
            if hasattr(owner, 'got_quat_distance_callback') and owner.got_quat_distance_callback:
                owner.got_quat_distance_callback(self.quat_distance)
            owner.step_count.quat = self.step_count


class SensorValuesData:
    """
    センサの値を格納するクラス

    Attributes:
        owner: オーナー（SensorValuesDataを生成したオブジェクトで、Orpheクラスのインスタンスが入っている。これはコールバック関数を登録するため）
        data: 生データ
        type: タイプ（40の場合は50Hzのv2, 50の場合は200Hzのv3）
        serial_number: シリアルナンバー
        timestamp: タイムスタンプ
        acc: 加速度センサの値
        converted_acc: 変換後の加速度センサの値
        gyro: ジャイロセンサの値
        converted_gyro: 変換後のジャイロセンサの値
        quat: クォータニオンの値
    """

    def __init__(self, owner, data, sensor_range):
        """
        コンストラクタ

        Args:
            owner: オーナー（SensorValuesDataを生成したオブジェクトで、Orpheクラスのインスタンスが入っている。これはコールバック関数を登録するため）
            data: 生データ
            sensor_range: 加速度センサとジャイロセンサのレンジ。Rangeクラスのインスタンス
            """
        if data[0] == 50:
            self.data = data
            self.type = int.from_bytes(
                data[0:1], byteorder='big', signed=False)
            self.serial_number = int.from_bytes(
                data[1:3], byteorder='big', signed=False)
            self.timestamp = to_timestamp(
                data[3], data[4], data[5], data[6], data[7])
            each_timestamp = self.timestamp

            for i in range(3, -1, -1):
                step = 21*i
                self.acc = AccData()
                self.acc.x = int.from_bytes(
                    data[22+step:24+step], byteorder='big', signed=True) / 32768
                self.acc.y = int.from_bytes(
                    data[24+step:26+step], byteorder='big', signed=True) / 32768
                self.acc.z = int.from_bytes(
                    data[26+step:28+step], byteorder='big', signed=True) / 32768

                self.converted_acc = AccData()
                amp_acc = [2, 4, 8, 16][sensor_range.acc]
                self.converted_acc.x = self.acc.x * amp_acc
                self.converted_acc.y = self.acc.y * amp_acc
                self.converted_acc.z = self.acc.z * amp_acc

                self.gyro = GyroData()
                self.gyro.x = int.from_bytes(
                    data[16+step:18+step], byteorder='big', signed=True) / 32768
                self.gyro.y = int.from_bytes(
                    data[18+step:20+step], byteorder='big', signed=True) / 32768
                self.gyro.z = int.from_bytes(
                    data[20+step:22+step], byteorder='big', signed=True) / 32768

                self.converted_gyro = GyroData()
                amp_gyro = [250, 500, 1000, 2000][sensor_range.gyro]
                self.converted_gyro.x = self.gyro.x * amp_gyro
                self.converted_gyro.y = self.gyro.y * amp_gyro
                self.converted_gyro.z = self.gyro.z * amp_gyro

                self.quat = QuatData()
                self.quat.w = int.from_bytes(
                    data[8+step:10+step], byteorder='big', signed=True) / 32768
                self.quat.x = int.from_bytes(
                    data[10+step:12+step], byteorder='big', signed=True) / 32768
                self.quat.y = int.from_bytes(
                    data[12+step:14+step], byteorder='big', signed=True) / 32768
                self.quat.z = int.from_bytes(
                    data[14+step:16+step], byteorder='big', signed=True) / 32768

                self.acc.serial_number = self.serial_number
                self.converted_acc.serial_number = self.serial_number
                self.gyro.serial_number = self.serial_number
                self.converted_gyro.serial_number = self.serial_number
                self.quat.serial_number = self.serial_number
                self.acc.packet_number = 3-i
                self.converted_acc.packet_number = 3-i
                self.gyro.packet_number = 3-i
                self.converted_gyro.packet_number = 3-i
                self.quat.packet_number = 3-i

                if i == 3:
                    self.acc.timestamp = each_timestamp
                    self.converted_acc.timestamp = each_timestamp
                    self.gyro.timestamp = each_timestamp
                    self.converted_gyro.timestamp = each_timestamp
                    self.quat.timestamp = each_timestamp
                else:
                    each_timestamp = each_timestamp + data[28+step]
                    self.acc.timestamp = each_timestamp
                    self.converted_acc.timestamp = each_timestamp
                    self.gyro.timestamp = each_timestamp
                    self.converted_gyro.timestamp = each_timestamp
                    self.quat.timestamp = each_timestamp

                # コールバック関数が設定されている場合、コールバック関数を呼び出す
                if hasattr(owner, 'got_acc_callback') and owner.got_acc_callback:
                    owner.got_acc_callback(self.acc)
                if hasattr(owner, 'got_converted_acc_callback') and owner.got_converted_acc_callback:
                    owner.got_converted_acc_callback(self.converted_acc)
                if hasattr(owner, 'got_gyro_callback') and owner.got_gyro_callback:
                    owner.got_gyro_callback(self.gyro)
                if hasattr(owner, 'got_converted_gyro_callback') and owner.got_converted_gyro_callback:
                    owner.got_converted_gyro_callback(self.converted_gyro)
                if hasattr(owner, 'got_quat_callback') and owner.got_quat_callback:
                    owner.got_quat_callback(self.quat)

        # 200Hz streaming gyro accel pressure
        elif data[0] == 55:
            self.data = data
            self.type = int.from_bytes(
                data[0:1], byteorder='big', signed=False)
            self.serial_number = int.from_bytes(
                data[1:3], byteorder='big', signed=False)
            self.timestamp = to_timestamp(
                data[3], data[4], data[5], data[6], data[7])
            each_timestamp = self.timestamp

            for i in range(3, -1, -1):
                step = 24*i
                self.acc = AccData()
                self.acc.x = int.from_bytes(
                    data[14+step:16+step], byteorder='big', signed=True) / 32768
                self.acc.y = int.from_bytes(
                    data[16+step:18+step], byteorder='big', signed=True) / 32768
                self.acc.z = int.from_bytes(
                    data[18+step:20+step], byteorder='big', signed=True) / 32768

                self.converted_acc = AccData()
                amp_acc = [2, 4, 8, 16][sensor_range.acc]
                self.converted_acc.x = self.acc.x * amp_acc
                self.converted_acc.y = self.acc.y * amp_acc
                self.converted_acc.z = self.acc.z * amp_acc

                self.gyro = GyroData()
                self.gyro.x = int.from_bytes(
                    data[8+step:10+step], byteorder='big', signed=True) / 32768
                self.gyro.y = int.from_bytes(
                    data[10+step:12+step], byteorder='big', signed=True) / 32768
                self.gyro.z = int.from_bytes(
                    data[12+step:14+step], byteorder='big', signed=True) / 32768

                self.converted_gyro = GyroData()
                amp_gyro = [250, 500, 1000, 2000][sensor_range.gyro]
                self.converted_gyro.x = self.gyro.x * amp_gyro
                self.converted_gyro.y = self.gyro.y * amp_gyro
                self.converted_gyro.z = self.gyro.z * amp_gyro

                # self.quat = QuatData()
                # self.quat.w = int.from_bytes(
                #     data[8+step:10+step], byteorder='big', signed=True) / 32768
                # self.quat.x = int.from_bytes(
                #     data[10+step:12+step], byteorder='big', signed=True) / 32768
                # self.quat.y = int.from_bytes(
                #     data[12+step:14+step], byteorder='big', signed=True) / 32768
                # self.quat.z = int.from_bytes(
                #     data[14+step:16+step], byteorder='big', signed=True) / 32768

                self.pressure = PressureData()
                for j in range(6):
                    self.pressure.values.append(int.from_bytes(
                        data[20+step+2*j:22+step+2*j], byteorder='big', signed=False))

                self.acc.serial_number = self.serial_number
                self.converted_acc.serial_number = self.serial_number
                self.gyro.serial_number = self.serial_number
                self.converted_gyro.serial_number = self.serial_number
                # self.quat.serial_number = self.serial_number
                self.pressure.serial_number = self.serial_number
                self.acc.packet_number = 3-i
                self.converted_acc.packet_number = 3-i
                self.gyro.packet_number = 3-i
                self.converted_gyro.packet_number = 3-i
                # self.quat.packet_number = 3-i
                self.pressure.packet_number = 3-i

                if i == 3:
                    self.acc.timestamp = each_timestamp
                    self.converted_acc.timestamp = each_timestamp
                    self.gyro.timestamp = each_timestamp
                    self.converted_gyro.timestamp = each_timestamp
                    # self.quat.timestamp = each_timestamp
                    self.pressure.timestamp = each_timestamp
                else:
                    each_timestamp = each_timestamp + data[28+step]
                    self.acc.timestamp = each_timestamp
                    self.converted_acc.timestamp = each_timestamp
                    self.gyro.timestamp = each_timestamp
                    self.converted_gyro.timestamp = each_timestamp
                    # self.quat.timestamp = each_timestamp
                    self.pressure.timestamp = each_timestamp

                # コールバック関数が設定されている場合、コールバック関数を呼び出す
                if hasattr(owner, 'got_acc_callback') and owner.got_acc_callback:
                    owner.got_acc_callback(self.acc)
                if hasattr(owner, 'got_converted_acc_callback') and owner.got_converted_acc_callback:
                    owner.got_converted_acc_callback(self.converted_acc)
                if hasattr(owner, 'got_gyro_callback') and owner.got_gyro_callback:
                    owner.got_gyro_callback(self.gyro)
                if hasattr(owner, 'got_converted_gyro_callback') and owner.got_converted_gyro_callback:
                    owner.got_converted_gyro_callback(self.converted_gyro)
                if hasattr(owner, 'got_pressure_callback') and owner.got_pressure_callback:
                    owner.got_pressure_callback(self.pressure)
                # if hasattr(owner, 'got_quat_callback') and owner.got_quat_callback:
                #     owner.got_quat_callback(self.quat)
         # 100Hz streaming gyro accel pressure quaternion
        # 100Hz streaming gyro accel pressure quaternion
        elif data[0] == 56:
            self.data = data
            self.type = int.from_bytes(
                data[0:1], byteorder='big', signed=False)
            self.serial_number = int.from_bytes(
                data[1:3], byteorder='big', signed=False)
            self.timestamp = to_timestamp(
                data[3], data[4], data[5], data[6], data[7])
            each_timestamp = self.timestamp

            for i in range(1, -1, -1):
                step = 32*i
                self.acc = AccData()
                self.acc.x = int.from_bytes(
                    data[22+step:24+step], byteorder='big', signed=True) / 32768
                self.acc.y = int.from_bytes(
                    data[24+step:26+step], byteorder='big', signed=True) / 32768
                self.acc.z = int.from_bytes(
                    data[26+step:28+step], byteorder='big', signed=True) / 32768

                self.converted_acc = AccData()
                amp_acc = [2, 4, 8, 16][sensor_range.acc]
                self.converted_acc.x = self.acc.x * amp_acc
                self.converted_acc.y = self.acc.y * amp_acc
                self.converted_acc.z = self.acc.z * amp_acc

                self.gyro = GyroData()
                self.gyro.x = int.from_bytes(
                    data[16+step:18+step], byteorder='big', signed=True) / 32768
                self.gyro.y = int.from_bytes(
                    data[18+step:20+step], byteorder='big', signed=True) / 32768
                self.gyro.z = int.from_bytes(
                    data[20+step:22+step], byteorder='big', signed=True) / 32768

                self.converted_gyro = GyroData()
                amp_gyro = [250, 500, 1000, 2000][sensor_range.gyro]
                self.converted_gyro.x = self.gyro.x * amp_gyro
                self.converted_gyro.y = self.gyro.y * amp_gyro
                self.converted_gyro.z = self.gyro.z * amp_gyro

                self.quat = QuatData()
                self.quat.w = int.from_bytes(
                    data[8+step:10+step], byteorder='big', signed=True) / 32768
                self.quat.x = int.from_bytes(
                    data[10+step:12+step], byteorder='big', signed=True) / 32768
                self.quat.y = int.from_bytes(
                    data[12+step:14+step], byteorder='big', signed=True) / 32768
                self.quat.z = int.from_bytes(
                    data[14+step:16+step], byteorder='big', signed=True) / 32768

                self.pressure = PressureData()
                for j in range(6):
                    self.pressure.values.append(int.from_bytes(
                        data[28+step+2*j:30+step+2*j], byteorder='big', signed=False))

                self.acc.serial_number = self.serial_number
                self.converted_acc.serial_number = self.serial_number
                self.gyro.serial_number = self.serial_number
                self.converted_gyro.serial_number = self.serial_number
                self.quat.serial_number = self.serial_number
                self.pressure.serial_number = self.serial_number
                self.acc.packet_number = 1-i
                self.converted_acc.packet_number = 1-i
                self.gyro.packet_number = 1-i
                self.converted_gyro.packet_number = 1-i
                self.quat.packet_number = 1-i
                self.pressure.packet_number = 1-i

                self.acc.timestamp = each_timestamp
                self.converted_acc.timestamp = each_timestamp
                self.gyro.timestamp = each_timestamp
                self.converted_gyro.timestamp = each_timestamp
                self.quat.timestamp = each_timestamp
                self.pressure.timestamp = each_timestamp

                # コールバック関数が設定されている場合、コールバック関数を呼び出す
                if hasattr(owner, 'got_acc_callback') and owner.got_acc_callback:
                    owner.got_acc_callback(self.acc)
                if hasattr(owner, 'got_converted_acc_callback') and owner.got_converted_acc_callback:
                    owner.got_converted_acc_callback(self.converted_acc)
                if hasattr(owner, 'got_gyro_callback') and owner.got_gyro_callback:
                    owner.got_gyro_callback(self.gyro)
                if hasattr(owner, 'got_converted_gyro_callback') and owner.got_converted_gyro_callback:
                    owner.got_converted_gyro_callback(self.converted_gyro)
                if hasattr(owner, 'got_pressure_callback') and owner.got_pressure_callback:
                    owner.got_pressure_callback(self.pressure)
                if hasattr(owner, 'got_quat_callback') and owner.got_quat_callback:
                    owner.got_quat_callback(self.quat)


class DeviceInformation:
    """
    デバイス情報を格納するクラス
    data: 生データ
    battery: バッテリー残量
    mount_position: マウント位置
        bit0 : 左右
        bit1 : 0(足底) / 1(足背)
        足底 : 左、右：0=(0000 0000b), 1=(0000 0001b)、
        足背 : 左、右：2(=0000 0010b), 3(=0000 0011b)
    range: レンジ
        加速度レンジ（ 2, 4, 8, 16(g)：0, 1, 2, 3）
        ジャイロレンジ（250, 500, 1000, 2000(°/s)：0, 1, 2, 3）
    version: ファームウェアバージョン
    device_information: 取得したデバイス情報の保管用メンバ変数
    """

    def __init__(self, data):
        self.data = data
        self.battery = int.from_bytes(data[0:1], byteorder='big', signed=False)
        self.mount_position = int.from_bytes(
            data[1:2], byteorder='big', signed=False)
        self.range = Range()
        self.range.acc = int.from_bytes(
            data[8:9], byteorder='big', signed=False)
        self.range.gyro = int.from_bytes(
            data[9:10], byteorder='big', signed=False)
        self.version = int.from_bytes(
            data[18:19], byteorder='big', signed=False)
        self.device_information = None


class StepCount:
    """
    歩数を格納するクラス"""

    def __init__(self):
        self.gait = 0
        self.stride = 0
        self.pronation = 0
        self.quat = 0


class Orphe:

    """
    ORPHE COREのBLE通信を行うクラス
    """

    def __init__(self):
        """
        コンストラクタ
        """
        self.serial_number_prev = 0
        self.client = None
        self.step_count = StepCount()  # 歩数

    def set_lost_data_callback(self, callback):
        """
        データが欠損したときに呼び出されるコールバック関数を設定する

        Args:
            callback: コールバック関数
        """
        self.lost_data_callback = callback

    def set_got_pressure_callback(self, callback):
        """
        圧力センサの値を取得したときに呼び出されるコールバック関数を設定する
        """
        self.got_pressure_callback = callback

    def set_got_acc_callback(self, callback):
        """加速度センサの値を取得したときに呼び出されるコールバック関数を設定する。

        例えば静止状態ではz方向の1Gの値はレンジ設定によって変わります。加速度レンジが2であれば 0.5 、16であれば 0.0625 （値は理論値なので誤差が生じます）です。
        """
        self.got_acc_callback = callback

    def set_got_gyro_callback(self, callback):
        """
        ジャイロセンサの値を取得したときに呼び出されるコールバック関数を設定する
        """
        self.got_gyro_callback = callback

    def set_got_converted_acc_callback(self, callback):
        """
        加速度センサの値を取得したときに呼び出されるコールバック関数を設定する

        それぞれの値は加速度レンジの値によって変換されます。
        例えば静止状態ではz方向の1Gの値は常に1.0です。
        """
        self.got_converted_acc_callback = callback

    def set_got_converted_gyro_callback(self, callback):
        """
        ジャイロセンサの値を取得したときに呼び出されるコールバック関数を設定する
        """
        self.got_converted_gyro_callback = callback

    def set_got_quat_callback(self, callback):
        """
        クォータニオンの値を取得したときに呼び出されるコールバック関数を設定する
        """
        self.got_quat_callback = callback

    def set_got_gait_callback(self, callback):
        """
        歩行解析の値を取得したときに呼び出されるコールバック関数を設定する
        """
        self.got_gait_callback = callback

    def set_got_stride_callback(self, callback):
        """
        ストライドの値を取得したときに呼び出されるコールバック関数を設定する
        """
        self.got_stride_callback = callback

    def set_got_pronation_callback(self, callback):
        """
        プロネーションの値を取得したときに呼び出されるコールバック関数を設定する
        """
        self.got_pronation_callback = callback

    def set_got_quat_distance_callback(self, callback):
        """
        クォータニオンと差分値を取得したときに呼び出されるコールバック関数を設定する
        """
        self.got_quat_distance_callback = callback

    def set_on_disconnect_callback(self, callback):
        """
        ORPHE COREとの接続が切断されたときに呼び出されるコールバック関数を設定する
        """
        self.on_disconnect_callback = callback

    async def scan_all_devices(self):
        """
        すべてのBLEデバイスをスキャンして、その結果を返す

        Returns:
            (BLEDevice, AdvertisementData) のタプルの辞書
        """
        print("Scanning all BLE devices...")
        devices = await BleakScanner.discover(return_adv=True)
        return devices

    async def connect(self, address=None):
        """
        ORPHE COREと接続する

        Args:
            address: 接続するデバイスのアドレス。指定しない場合はスキャンしてSERVICE UUIDで合致するものに接続する
        Returns:
            接続に成功した場合はTrue、失敗した場合はFalse
        """
        print(
            f"Scanning for ORPHE CORE BLE device...[address specified: {address}]")
        devices = await BleakScanner.discover(return_adv=True)

        target_device = None

        if address is not None:
            for addr, (device, adv_data) in devices.items():
                if device.address == address:
                    target_device = device
                    print(
                        f"Found target device: {device.name}(name), {device.address}(address)")
                    break
        else:
            for addr, (device, adv_data) in devices.items():
                #  device.name に INS* が含まれている場合に接続する
                if device.name is not None and "INS" in device.name:
                    target_device = device
                    print(
                        f"Found target device: {device.name}(name), {device.address}(address)")
                    break

        if target_device is None:
            print("Target device not found.")
            return False

        self.client = BleakClient(target_device.address)
        await self.client.connect()
        if self.client.is_connected:
            print("Connected to the device")

            # 別のタスクで接続状態を監視
            asyncio.create_task(self.monitor_connection(self.client))

            return True
        else:
            print("Failed to connect to the device")
            return False

    async def monitor_connection(self, client):
        while True:
            await asyncio.sleep(1)  # 1秒ごとに接続状態をチェック
            if not client.is_connected:
                await self.disconnect_callback(self)
                break

    async def disconnect_callback(self, owner):
        # コールバック関数が設定されている場合、コールバック関数を呼び出す
        if hasattr(owner, 'on_disconnect_callback') and owner.on_disconnect_callback:
            owner.on_disconnect_callback()

    def is_connected(self):
        """
        ORPHE COREに接続されているかどうかを返す

        Returns:
            接続されている場合はTrue、されていない場合はFalse
        """
        if self.client is None:
            return False
        return self.client.is_connected

    async def read_device_information(self):
        """
        ORPHE COREのデバイス情報を取得する。一度取得したデバイス情報はself.device_informationメンバ変数に保存される。
        Returns: DeviceInformationクラスのインスタンス
        """
        di = await self.client.read_gatt_char(CHARACTERISTIC_DEVICE_INFORMATION_UUID)
        di = DeviceInformation(di)
        self.device_information = di  # デバイス情報をメンバ変数として保存（更新）しておく
        return di

    async def print_device_information(self):
        """
        ORPHE COREのデバイス情報を取得し、標準出力に表示する。ほぼデバッグ用途
        """
        di = await self.read_device_information()
        print(f"Version: {di.version}")
        print(f"Battery: {di.battery}")
        print(f"Mount Position: {di.mount_position}")
        print(f"ACC Range: {di.range.acc}")
        print(f"GYRO Range: {di.range.gyro}")
        # print(f"Raw Data: {list(self.device_information.data)}")

    async def set_mount_position(self, mount_position):
        """
        mount_position(int): 0 or 1
        Returns: None
        """
        # mount_positionの値の範囲をチェック
        if mount_position < 0 or mount_position > 3:
            print("mount_position must be 0, 1, 2, 3.")
            return

        di = await self.read_device_information()
        di.mount_position = mount_position
        await self.write_device_information()

    async def getChecksum(self, data):
        """
        デバイス情報のチェックサムを計算する
        """
        checksum = 0
        for i in range(0, 19):
            checksum += data[i]
        checksum = checksum & 0xFF
        return checksum

    async def right_dec2hex(self, value):
        """
        10進数を16進数に変換し、右から2桁を返す
        """
        # 16進数(大文字)で4桁ゼロ埋め
        hex_str = format(value, '04X')
        # 右2文字を返す
        return hex_str[-2:]

    async def write_device_information(self):
        """
        this.device_information に設定されている内容をORPHE INSOLEに書き込む。
        書き込み後にすぐデバイスインフォメーションを読み込むと正しいデータが取得できないため、WRITE_WAIT_INTERVAL_SEC秒待つ
        """
        if self.device_information is None:
            print("Device information is not set, Please read device information first.")
            return

        ba_write = bytearray([
            0x09,
            self.device_information.mount_position,
            0x00,
            0x00,
            0x01,
            0x00,
            0x3C,
            self.device_information.range.acc,
            self.device_information.range.gyro,
            0x00,
            0x00,
            0x00,
            0xFF,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            self.device_information.version,
        ])

        checksum = await self.getChecksum(ba_write)
        checksum_hex = await self.right_dec2hex(checksum)
        ba_write = ba_write + bytearray([checksum])  # 19バイト目にチェックサムを設定

        print(f"data: {list(ba_write)}, size: {len(ba_write)}")
        print(f"checksum: {checksum}, hex: {checksum_hex}")

        await self.client.write_gatt_char(CHARACTERISTIC_DEVICE_INFORMATION_UUID, ba_write)

        # 100ms待つ（これがないと即座にdevice informationを読み込まれると正しいデータ取得ができないため）
        await asyncio.sleep(WRITE_WAIT_INTERVAL_SEC)

    async def set_acc_range(self, acc_range):
        """
        acc_range(int): 2,4,8,16G を順番に 0,1,2,3 で指定
        Returns: None
        """
        # acc_rangeの値は2,4,8,16のいずれかなので、チェックする
        if acc_range != 2 and acc_range != 4 and acc_range != 8 and acc_range != 16:
            print("acc_range must be 2, 4, 8, or 16[g].")
            return

        # acc_range を 0,1,2,3 に変換
        acc_range = [2, 4, 8, 16].index(acc_range)

        # デバイス情報を読み込む
        di = await self.read_device_information()

        # デバイス情報のレンジ設定を変更
        di.range.acc = acc_range

        # デバイス情報を書き込む
        await self.write_device_information()

    async def set_data_streaming_mode(self, mode=4):
        """
        データストリーミングモードを設定する

        mode(int): 1,3,4 のいずれか
            0x0D,0x01：リアルタイムデータを取得（従来の200Hzデータ形式）
            0x0D,0x03: リアルタイム（ジャイロ、加速度、圧力）200Hz
            0x0D,0x04: リアルタイム（ジャイロ、加速度、圧力、クオータニオン）100Hz -- デフォルト
        Returns: None
        """
        if mode != 1 and mode != 3 and mode != 4:
            print("mode must be 1, 3, or 4.")
            return
        ba_write = bytearray([0x0D, mode])
        await self.client.write_gatt_char(CHARACTERISTIC_DEVICE_INFORMATION_UUID, ba_write)
        # 指定ms待つ（これがないと即座にdevice informationを読み込まれると正しいデータ取得ができないため）
        await asyncio.sleep(WRITE_WAIT_INTERVAL_SEC)

    async def set_gyro_range(self, gyro_range):
        """
        gyro_range(int): 250,500,1000,2000[deg/s] を順番に 0,1,2,3 で指定
        Returns: None
        """
        # acc_rangeの値は2,4,8,16のいずれかなので、チェックする
        if gyro_range != 250 and gyro_range != 500 and gyro_range != 1000 and gyro_range != 2000:
            print("gyro_range must be 250, 500, 1000, or 2000[deg/s].")
            return

        # gyro_range を 0,1,2,3 に変換
        gyro_range = [250, 500, 1000, 2000].index(gyro_range)

        # デバイス情報を読み込む
        di = await self.read_device_information()

        # デバイス情報のレンジ設定を変更
        di.range.gyro = gyro_range

        await self.write_device_information()

    async def sensor_values_notification_handler(self, sender, data):
        """
        センサの値を取得したときに呼び出されるハンドラ
        """
        if (self.is_connected() == False):
            return

        # データの長さを確認
        if data[0] == 50 or data[0] == 55 or data[0] == 56:
            sensor_values = SensorValuesData(
                self, data, self.device_information.range)
            if (sensor_values.serial_number - self.serial_number_prev) > 1:
                # データ欠損の場合
                # コールバック関数が設定されている場合、コールバック関数を呼び出す
                if hasattr(self, 'lost_data_callback') and self.lost_data_callback:
                    self.lost_data_callback(
                        self.serial_number_prev, sensor_values.serial_number)
            self.serial_number_prev = sensor_values.serial_number
        elif data[0] == 40:
            sensor_values = SensorValuesData(
                self, data, self.device_information.range)

    async def start_sensor_values_notification(self):
        """
        センサの値の通知を開始する。ただしセンサ値のレンジを取得しておかないといけないので、最初にデバイス情報を取得する
        """
        await self.read_device_information()
        await self.client.start_notify(CHARACTERISTIC_SENSOR_VALUES_UUID, self.sensor_values_notification_handler)

    async def step_analysis_notification_handler(self, sender, data):
        """
        ステップ解析の値を取得したときに呼び出されるハンドラ
        """
        # print(f"Step analysis: {data[1]}")
        StepAnalysisData(self, data)

    async def start_step_analysis_notification(self):
        """
        ステップ解析の通知を開始する
        """
        await self.client.start_notify(CHARACTERISTIC_STEP_ANALYSIS_UUID, self.step_analysis_notification_handler)

    async def stop_sensor_values_notification(self):
        """
        センサの値の通知を停止する
        """
        await self.client.stop_notify(CHARACTERISTIC_SENSOR_VALUES_UUID)

    async def stop_step_analysis_notification(self):
        """
        ステップ解析の通知を停止する
        """
        await self.client.stop_notify(CHARACTERISTIC_STEP_ANALYSIS_UUID)

    async def disconnect(self):
        """
        ORPHE COREとの接続を切断する
        """
        await self.client.disconnect()
