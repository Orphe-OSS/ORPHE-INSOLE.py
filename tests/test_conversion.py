"""
センサ値の換算（スケール）を固定する単体テスト

合成したSENSOR_VALUESパケットを SensorValuesData に通し、
  * ジャイロ物理値 = raw * フルスケール[dps] * 0.000035（IMUの実感度）
  * クォータニオン = raw / 16384（Q14）
  * 加速度物理値 = raw / 32768 * フルスケール[G]（変更なし）
  * 正規化値（acc/gyro）= raw / 32768（後方互換のため変更なし）
を検証する。BLE（bleak）は不要。

実行方法:
    python3 -m unittest discover -s tests
    python3 tests/test_conversion.py
"""

import math
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# bleak が未インストールの環境でも換算ロジックだけをテストできるようにする
try:
    import bleak  # noqa: F401
except ModuleNotFoundError:  # pragma: no cover
    _bleak_stub = types.ModuleType("bleak")
    _bleak_stub.BleakClient = object
    _bleak_stub.BleakScanner = object
    sys.modules["bleak"] = _bleak_stub

from orphe_insole import (  # noqa: E402
    ACC_RANGES,
    GYRO_DPS_PER_LSB_PER_RANGE,
    GYRO_RANGES,
    GYRO_SENSITIVITY_SCALE,
    QUAT_SCALE,
    Range,
    SensorValuesData,
)


# ---------------------------------------------------------------- ヘルパー

def _i16(value):
    return int(value).to_bytes(2, byteorder="big", signed=True)


def _u16(value):
    return int(value).to_bytes(2, byteorder="big", signed=False)


PACKET_SIZE = 104
# data[3..7]: 時, 分, 秒, ミリ秒上位, ミリ秒下位
HEADER_TIME = bytes([12, 30, 15, 0, 100])


def _pad(payload):
    if len(payload) > PACKET_SIZE:
        raise ValueError(f"packet too long: {len(payload)}")
    return payload + bytes(PACKET_SIZE - len(payload))


def build_packet_50(quat, gyro, acc, frames=4, serial=1):
    """ヘッダ50（mode 1: quat+gyro+acc, フレーム間隔21バイト）のパケットを合成する"""
    payload = bytes([50]) + _u16(serial) + HEADER_TIME
    for _ in range(frames):
        payload += b"".join(_i16(v) for v in quat)   # w, x, y, z
        payload += b"".join(_i16(v) for v in gyro)   # x, y, z
        payload += b"".join(_i16(v) for v in acc)    # x, y, z
        payload += bytes([5])                        # 次フレームまでの経過ms
    return _pad(payload)


def build_packet_55(gyro, acc, press, frames=4, serial=2):
    """ヘッダ55（mode 3: gyro+acc+press, フレーム間隔24バイト）のパケットを合成する"""
    payload = bytes([55]) + _u16(serial) + HEADER_TIME
    for _ in range(frames):
        payload += b"".join(_i16(v) for v in gyro)
        payload += b"".join(_i16(v) for v in acc)
        payload += b"".join(_u16(v) for v in press)
    return _pad(payload)


def build_packet_56(quat, gyro, acc, press, frames=2, serial=3):
    """ヘッダ56（mode 4: quat+gyro+acc+press, フレーム間隔32バイト）のパケットを合成する"""
    payload = bytes([56]) + _u16(serial) + HEADER_TIME
    for _ in range(frames):
        payload += b"".join(_i16(v) for v in quat)
        payload += b"".join(_i16(v) for v in gyro)
        payload += b"".join(_i16(v) for v in acc)
        payload += b"".join(_u16(v) for v in press)
    return _pad(payload)


class Collector:
    """コールバックで受け取った値を溜めるだけのオーナー役"""

    def __init__(self):
        self.acc = []
        self.converted_acc = []
        self.gyro = []
        self.converted_gyro = []
        self.quat = []
        self.pressure = []

    def got_acc_callback(self, acc):
        self.acc.append(acc)

    def got_converted_acc_callback(self, acc):
        self.converted_acc.append(acc)

    def got_gyro_callback(self, gyro):
        self.gyro.append(gyro)

    def got_converted_gyro_callback(self, gyro):
        self.converted_gyro.append(gyro)

    def got_quat_callback(self, quat):
        self.quat.append(quat)

    def got_pressure_callback(self, pressure):
        self.pressure.append(pressure)


def parse(data, acc_range_index=0, gyro_range_index=0):
    """合成パケットをパースして Collector を返す"""
    sensor_range = Range()
    sensor_range.acc = acc_range_index
    sensor_range.gyro = gyro_range_index
    collector = Collector()
    SensorValuesData(collector, data, sensor_range)
    return collector


# ---------------------------------------------------------------- 定数

class TestConstants(unittest.TestCase):

    def test_gyro_sensitivity_scale(self):
        # IMU(LSM6DSOX)の実感度: レンジ × 0.000035 [dps/LSB]
        self.assertAlmostEqual(GYRO_DPS_PER_LSB_PER_RANGE, 0.000035)
        self.assertAlmostEqual(GYRO_SENSITIVITY_SCALE, 1.14688)
        # 理想Q15換算は実感度より約12.8%小さい
        self.assertAlmostEqual(1 / GYRO_SENSITIVITY_SCALE, 0.8719, places=4)
        # ±2000dps は 70 mdps/LSB
        self.assertAlmostEqual(GYRO_RANGES[3] * GYRO_DPS_PER_LSB_PER_RANGE, 0.07)

    def test_quat_scale_is_q14(self):
        self.assertEqual(QUAT_SCALE, 16384)


# ---------------------------------------------------------------- ジャイロ

class TestGyroConversion(unittest.TestCase):

    RAW = (1000, -2000, 16384)

    def _expected(self, raw, gyro_range_index):
        # 実感度[dps/LSB] × raw
        return raw * GYRO_RANGES[gyro_range_index] * GYRO_DPS_PER_LSB_PER_RANGE

    def test_all_ranges_header_56(self):
        for gyro_range_index in range(4):
            with self.subTest(gyro_range=GYRO_RANGES[gyro_range_index]):
                data = build_packet_56(
                    quat=(16384, 0, 0, 0), gyro=self.RAW, acc=(0, 0, 0),
                    press=(0,) * 6)
                got = parse(data, gyro_range_index=gyro_range_index)
                self.assertEqual(len(got.converted_gyro), 2)
                for sample in got.converted_gyro:
                    self.assertAlmostEqual(
                        sample.x, self._expected(self.RAW[0], gyro_range_index))
                    self.assertAlmostEqual(
                        sample.y, self._expected(self.RAW[1], gyro_range_index))
                    self.assertAlmostEqual(
                        sample.z, self._expected(self.RAW[2], gyro_range_index))

    def test_all_headers_same_sensitivity(self):
        packets = {
            50: build_packet_50(quat=(16384, 0, 0, 0), gyro=self.RAW,
                                acc=(0, 0, 0)),
            55: build_packet_55(gyro=self.RAW, acc=(0, 0, 0), press=(0,) * 6),
            56: build_packet_56(quat=(16384, 0, 0, 0), gyro=self.RAW,
                                acc=(0, 0, 0), press=(0,) * 6),
        }
        for header, data in packets.items():
            with self.subTest(header=header):
                got = parse(data, gyro_range_index=3)  # ±2000 dps
                self.assertTrue(got.converted_gyro)
                for sample in got.converted_gyro:
                    # ±2000dps なら 0.07 dps/LSB
                    self.assertAlmostEqual(sample.x, self.RAW[0] * 0.07)
                    self.assertAlmostEqual(sample.y, self.RAW[1] * 0.07)
                    self.assertAlmostEqual(sample.z, self.RAW[2] * 0.07)

    def test_full_scale_raw_reads_above_nominal_range(self):
        # フルスケールのraw(32767)は理想Q15なら約2000dps、実感度なら約2293dps
        data = build_packet_56(quat=(16384, 0, 0, 0), gyro=(32767, 0, 0),
                               acc=(0, 0, 0), press=(0,) * 6)
        got = parse(data, gyro_range_index=3)
        self.assertAlmostEqual(got.converted_gyro[0].x, 32767 * 0.07, places=3)
        self.assertAlmostEqual(got.converted_gyro[0].x / 2000, 1.14684,
                               places=4)

    def test_regression_against_ideal_q15(self):
        # 旧実装（raw/32768*range）に対して常に 1.14688 倍になっていること
        data = build_packet_56(quat=(16384, 0, 0, 0), gyro=self.RAW,
                               acc=(0, 0, 0), press=(0,) * 6)
        got = parse(data, gyro_range_index=2)  # ±1000 dps
        legacy = self.RAW[0] / 32768 * GYRO_RANGES[2]
        self.assertAlmostEqual(got.converted_gyro[0].x / legacy,
                               GYRO_SENSITIVITY_SCALE)

    def test_normalized_gyro_unchanged(self):
        # 正規化値は後方互換のため raw/32768 のまま
        data = build_packet_56(quat=(16384, 0, 0, 0), gyro=self.RAW,
                               acc=(0, 0, 0), press=(0,) * 6)
        got = parse(data, gyro_range_index=3)
        for sample in got.gyro:
            self.assertAlmostEqual(sample.x, self.RAW[0] / 32768)
            self.assertAlmostEqual(sample.y, self.RAW[1] / 32768)
            self.assertAlmostEqual(sample.z, self.RAW[2] / 32768)


# ---------------------------------------------------------------- クォータニオン

class TestQuatConversion(unittest.TestCase):

    def test_identity_quaternion_norm_is_one(self):
        data = build_packet_56(quat=(16384, 0, 0, 0), gyro=(0, 0, 0),
                               acc=(0, 0, 0), press=(0,) * 6)
        got = parse(data)
        self.assertEqual(len(got.quat), 2)
        for quat in got.quat:
            self.assertAlmostEqual(quat.w, 1.0)
            self.assertAlmostEqual(quat.x, 0.0)
            self.assertAlmostEqual(quat.y, 0.0)
            self.assertAlmostEqual(quat.z, 0.0)

    def test_rotated_quaternion_norm_is_one(self):
        # z軸まわり45度: w=cos(22.5deg), z=sin(22.5deg) を Q14 で符号化
        w_raw = round(math.cos(math.radians(22.5)) * QUAT_SCALE)
        z_raw = round(math.sin(math.radians(22.5)) * QUAT_SCALE)
        for header, data in (
            (50, build_packet_50(quat=(w_raw, 0, 0, z_raw), gyro=(0, 0, 0),
                                 acc=(0, 0, 0))),
            (56, build_packet_56(quat=(w_raw, 0, 0, z_raw), gyro=(0, 0, 0),
                                 acc=(0, 0, 0), press=(0,) * 6)),
        ):
            with self.subTest(header=header):
                got = parse(data)
                self.assertTrue(got.quat)
                for quat in got.quat:
                    norm = math.sqrt(quat.w ** 2 + quat.x ** 2
                                     + quat.y ** 2 + quat.z ** 2)
                    self.assertAlmostEqual(norm, 1.0, places=4)
                    # yaw = 45度
                    yaw = math.degrees(2 * math.atan2(quat.z, quat.w))
                    self.assertAlmostEqual(yaw, 45.0, places=2)

    def test_legacy_q15_would_halve_the_norm(self):
        # 旧実装（/32768）ではノルムが約0.5に潰れていたことを明示
        data = build_packet_56(quat=(16384, 0, 0, 0), gyro=(0, 0, 0),
                               acc=(0, 0, 0), press=(0,) * 6)
        got = parse(data)
        legacy_w = 16384 / 32768
        self.assertAlmostEqual(legacy_w, 0.5)
        self.assertAlmostEqual(got.quat[0].w / legacy_w, 2.0)


# ---------------------------------------------------------------- 加速度・圧力

class TestAccAndPressureUnchanged(unittest.TestCase):

    RAW = (16384, -8192, 4096)

    def test_converted_acc_uses_range_only(self):
        for acc_range_index in range(4):
            with self.subTest(acc_range=ACC_RANGES[acc_range_index]):
                data = build_packet_56(quat=(16384, 0, 0, 0), gyro=(0, 0, 0),
                                       acc=self.RAW, press=(0,) * 6)
                got = parse(data, acc_range_index=acc_range_index)
                amp = ACC_RANGES[acc_range_index]
                for sample in got.converted_acc:
                    self.assertAlmostEqual(sample.x, self.RAW[0] / 32768 * amp)
                    self.assertAlmostEqual(sample.y, self.RAW[1] / 32768 * amp)
                    self.assertAlmostEqual(sample.z, self.RAW[2] / 32768 * amp)

    def test_static_1g_is_one(self):
        # 静止時 z に 1G がかかっている場合、レンジに依らず converted は 1.0
        for acc_range_index in range(4):
            amp = ACC_RANGES[acc_range_index]
            raw_z = round(32768 / amp)
            data = build_packet_56(quat=(16384, 0, 0, 0), gyro=(0, 0, 0),
                                   acc=(0, 0, raw_z), press=(0,) * 6)
            got = parse(data, acc_range_index=acc_range_index)
            self.assertAlmostEqual(got.converted_acc[0].z, 1.0, places=3)

    def test_normalized_acc_unchanged(self):
        data = build_packet_56(quat=(16384, 0, 0, 0), gyro=(0, 0, 0),
                               acc=self.RAW, press=(0,) * 6)
        got = parse(data, acc_range_index=3)
        for sample in got.acc:
            self.assertAlmostEqual(sample.x, self.RAW[0] / 32768)
            self.assertAlmostEqual(sample.y, self.RAW[1] / 32768)
            self.assertAlmostEqual(sample.z, self.RAW[2] / 32768)

    def test_pressure_values_are_raw_adc(self):
        press = (0, 1, 1000, 30000, 60000, 65535)
        for header, data, frames in (
            (55, build_packet_55(gyro=(0, 0, 0), acc=(0, 0, 0), press=press), 4),
            (56, build_packet_56(quat=(16384, 0, 0, 0), gyro=(0, 0, 0),
                                 acc=(0, 0, 0), press=press), 2),
        ):
            with self.subTest(header=header):
                got = parse(data)
                self.assertEqual(len(got.pressure), frames)
                for sample in got.pressure:
                    self.assertEqual(tuple(sample.values), press)


if __name__ == "__main__":
    unittest.main(verbosity=2)
