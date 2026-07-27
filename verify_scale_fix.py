"""
実機でジャイロ感度とクォータニオンスケールの修正を検証するスクリプト

確認できること:
  1. クォータニオンのノルム（Q14 = /16384 が正しければ 1.0 になる）
  2. ジャイロ物理値[deg/s]（IMUの実感度で換算されているか）
  3. 90度回転させたときの「ジャイロz積分角」と「クォータニオンyaw変化」の一致

使い方:
    python verify_scale_fix.py
  インソールを机の上に平らに置いた状態で起動し、画面の指示に従ってください。
  終了する場合は Ctrl+C で終了できます。
"""

import asyncio
import math
import time

from bleak import BleakScanner

from orphe_insole import (
    GYRO_DPS_PER_LSB_PER_RANGE,
    GYRO_RANGES,
    GYRO_SENSITIVITY_SCALE,
    QUAT_SCALE,
    SERVICE_ORPHE_INFORMATION_UUID,
    Orphe,
)


async def find_insole_address():
    """ORPHEサービスUUIDのアドバタイズでINSOLEを探してアドレスを返す。

    ライブラリ既定の探索はデバイス名に "INS" を含むものを探すが、
    macOS(CoreBluetooth)はアドバタイズ名を返さないことが多く
    （scan結果の name が None になる）、名前マッチでは見つからない。
    Web Bluetooth(JS版)と同様にサービスUUIDで判定する。
    """
    print("Scanning for ORPHE INSOLE (by service UUID, 10s)...")
    devices = await BleakScanner.discover(timeout=10.0, return_adv=True)
    candidates = []
    for _addr, (device, adv) in devices.items():
        uuids = [u.lower() for u in (adv.service_uuids or [])]
        name = device.name or adv.local_name or ""
        if SERVICE_ORPHE_INFORMATION_UUID.lower() in uuids or "INS" in name:
            candidates.append((adv.rssi, device, name))
    if not candidates:
        print("ORPHE INSOLE が見つかりません。以下を確認してください:")
        print("  - Chrome等の他アプリが接続したままになっていないか（接続中はアドバタイズ停止）")
        print("  - INSOLEがスリープしていないか（軽く動かす）")
        return None
    candidates.sort(reverse=True, key=lambda c: c[0])
    for rssi, device, name in candidates:
        print(f"  found: {name or '(no name)'} rssi={rssi} address={device.address}")
    best = candidates[0]
    print(f"Connecting to {best[1].address} (rssi={best[0]})")
    return best[1].address

# 回転テストの合否しきい値（ジャイロ積分角 / yaw変化）
RATIO_PASS_MIN = 0.93
RATIO_PASS_MAX = 1.07
# 修正が効いていない場合の比率（理想Q15換算 / 実感度換算 = 1 / 1.14688）
RATIO_LEGACY = 1 / GYRO_SENSITIVITY_SCALE  # ≒ 0.872
# 机の上で回してもらう基準角度[deg]
REFERENCE_ANGLE_DEG = 90.0


def quat_norm(quat):
    """クォータニオンのノルムを返す"""
    return math.sqrt(quat.w ** 2 + quat.x ** 2 + quat.y ** 2 + quat.z ** 2)


def quat_yaw_deg(quat):
    """クォータニオンから yaw[deg] を求める（-180〜180）"""
    siny = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy = 1.0 - 2.0 * (quat.y ** 2 + quat.z ** 2)
    return math.degrees(math.atan2(siny, cosy))


def wrap_deg(angle):
    """角度差を -180〜180 に畳み込む"""
    return (angle + 180.0) % 360.0 - 180.0


class ScaleVerifier:
    """センサ値を受け取って統計と積分値を保持するクラス"""

    def __init__(self):
        # ライブ表示用
        self.quat_norm_latest = None
        self.quat_norm_sum = 0.0
        self.quat_norm_count = 0
        self.gyro_latest = None
        self.gyro_count = 0

        # 回転テスト用
        self.recording = False
        self.gyro_z_sum = 0.0        # [deg/s] の総和（後で dt を掛ける）
        self.gyro_sample_count = 0
        self.record_started_at = 0.0
        self.record_elapsed = 0.0
        self.yaw_prev = None
        self.yaw_unwrapped = 0.0     # yawの累積変化[deg]

    # ---------------------------------------------------------- コールバック

    def got_quat(self, quat):
        norm = quat_norm(quat)
        self.quat_norm_latest = norm
        self.quat_norm_sum += norm
        self.quat_norm_count += 1

        # ノルムが0に近いときは yaw が定義できないのでスキップ
        if norm < 1e-6:
            return
        yaw = quat_yaw_deg(quat)
        if self.recording:
            if self.yaw_prev is not None:
                self.yaw_unwrapped += wrap_deg(yaw - self.yaw_prev)
            self.yaw_prev = yaw
        else:
            self.yaw_prev = yaw

    def got_converted_gyro(self, gyro):
        self.gyro_latest = (gyro.x, gyro.y, gyro.z)
        self.gyro_count += 1
        if self.recording:
            self.gyro_z_sum += gyro.z
            self.gyro_sample_count += 1

    # ---------------------------------------------------------- 回転テスト

    def start_recording(self):
        self.recording = False
        self.gyro_z_sum = 0.0
        self.gyro_sample_count = 0
        self.yaw_unwrapped = 0.0
        self.yaw_prev = None
        self.record_started_at = time.monotonic()
        self.recording = True

    def stop_recording(self):
        self.recording = False
        self.record_elapsed = time.monotonic() - self.record_started_at

    def gyro_integral_deg(self):
        """ジャイロz軸の積分角[deg]。dt は実測サンプルレートから求める"""
        if self.gyro_sample_count == 0 or self.record_elapsed <= 0:
            return 0.0
        dt = self.record_elapsed / self.gyro_sample_count
        return self.gyro_z_sum * dt

    def sample_rate_hz(self):
        if self.record_elapsed <= 0:
            return 0.0
        return self.gyro_sample_count / self.record_elapsed


async def ask_enter(prompt):
    """イベントループを止めずに Enter 入力を待つ"""
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, input, prompt)


async def live_monitor(verifier, stop_event):
    """quatノルムとジャイロ物理値を定期表示する"""
    while not stop_event.is_set():
        await asyncio.sleep(0.2)
        if verifier.quat_norm_latest is None or verifier.gyro_latest is None:
            print("\rデータ待ち... (mode 4 で quat と gyro が来るまで待機)",
                  end="", flush=True)
            continue
        norm = verifier.quat_norm_latest
        mean = verifier.quat_norm_sum / max(1, verifier.quat_norm_count)
        gx, gy, gz = verifier.gyro_latest
        print(
            f"\rquatノルム: {norm:6.3f} (平均 {mean:6.3f} / 旧換算 /32768 なら {mean/2:5.3f})"
            f"  gyro[dps]: {gx:8.2f}, {gy:8.2f}, {gz:8.2f}   ",
            end="", flush=True)


def print_norm_judgement(verifier):
    """quatノルムの判定を表示する"""
    if verifier.quat_norm_count == 0:
        print("クォータニオンを受信できませんでした。streaming mode 4 で接続できているか確認してください。")
        return
    mean = verifier.quat_norm_sum / verifier.quat_norm_count
    print(f"\nquatノルム平均: {mean:.4f}  (サンプル {verifier.quat_norm_count}件, "
          f"現在の QUAT_SCALE = {QUAT_SCALE})")
    if 0.9 <= mean <= 1.1:
        print("  => PASS: ノルム≈1.0。Q14（/16384）への修正は正しいです。")
    elif 0.4 <= mean <= 0.6:
        print("  => NG: ノルム≈0.5。/16384 でも小さすぎます（Q13相当？）。FW仕様を再確認してください。")
    elif 1.8 <= mean <= 2.2:
        print("  => NG: ノルム≈2.0。旧換算の /32768（Q15）が正しいので QUAT_SCALE を 32768 に revert してください。")
    else:
        print("  => 判定不能: 静止させて再測定してください（動かすとFWのフィルタでノルムが乱れる場合があります）。")


def print_rotation_result(verifier):
    """回転テストの結果を表示する"""
    integral = verifier.gyro_integral_deg()
    yaw = verifier.yaw_unwrapped
    legacy_integral = integral / GYRO_SENSITIVITY_SCALE  # 旧換算（理想Q15）での積分値

    print("\n--- 回転テスト結果 ---")
    print(f"収録時間          : {verifier.record_elapsed:.2f} s "
          f"({verifier.gyro_sample_count} サンプル, 実測 {verifier.sample_rate_hz():.1f} Hz)")
    print(f"ジャイロz積分角   : {integral:+8.2f} deg  (現在の実感度換算)")
    print(f"  参考: 旧換算    : {legacy_integral:+8.2f} deg  (raw/32768*range)")
    print(f"quat yaw変化      : {yaw:+8.2f} deg")
    print(f"基準（手で回した角度）: {REFERENCE_ANGLE_DEG:.0f} deg")

    if abs(yaw) < 10.0:
        print("\n  => 判定不能: yaw変化が小さすぎます。水平に置いた状態で90度しっかり回してください。")
        return
    if abs(integral) < 10.0:
        print("\n  => 判定不能: ジャイロ積分角が小さすぎます。ゆっくり回しすぎていないか確認してください。")
        return

    ratio = abs(integral) / abs(yaw)
    print(f"比率 (ジャイロ積分 / yaw変化): {ratio:.3f}")
    if RATIO_PASS_MIN <= ratio <= RATIO_PASS_MAX:
        print(f"  => PASS: 比率が {RATIO_PASS_MIN}〜{RATIO_PASS_MAX} に収まっています。"
              "ジャイロ感度の修正は正しく効いています。")
    elif abs(ratio - RATIO_LEGACY) < 0.04:
        print(f"  => NG: 比率が {RATIO_LEGACY:.3f} 付近です（=修正が効いていない）。"
              "converted_gyro に GYRO_SENSITIVITY_SCALE が掛かっているか確認してください。")
    elif abs(ratio - GYRO_SENSITIVITY_SCALE) < 0.05:
        print(f"  => NG: 比率が {GYRO_SENSITIVITY_SCALE:.3f} 付近です（=補正が二重に掛かっている可能性）。")
    else:
        print("  => 判定不能: 回転が速すぎる/遅すぎる、または軸がずれています。"
              "机に平らに置き、垂直軸まわりに一定速度で90度回して再測定してください。")


async def main():
    print("=== ORPHE INSOLE 換算スケール検証 ===")
    print(f"ジャイロ実感度   : レンジ × {GYRO_DPS_PER_LSB_PER_RANGE} dps/LSB "
          f"(補正係数 {GYRO_SENSITIVITY_SCALE:.5f})")
    print(f"クォータニオン   : raw / {QUAT_SCALE}")
    print()

    verifier = ScaleVerifier()
    orphe = Orphe()

    orphe.set_got_quat_callback(verifier.got_quat)
    orphe.set_got_converted_gyro_callback(verifier.got_converted_gyro)

    address = await find_insole_address()
    if address is None:
        return
    if not await orphe.connect(address=address):
        return

    # quaternionが必要なので mode 4（gyro + acc + press + quat, 100Hz）
    await orphe.set_data_streaming_mode(4)
    di = await orphe.read_device_information()
    print(f"ジャイロレンジ設定: ±{GYRO_RANGES[di.range.gyro]} dps "
          f"(感度 {GYRO_RANGES[di.range.gyro] * GYRO_DPS_PER_LSB_PER_RANGE * 1000:.2f} mdps/LSB)")
    await orphe.start_sensor_values_notification()

    stop_event = asyncio.Event()
    monitor_task = asyncio.create_task(live_monitor(verifier, stop_event))

    try:
        print("\n[1] インソールを机の上に平らに置いて静止させてください。")
        print("    quatノルムが 1.0 に近ければ Q14（/16384）が正しく、")
        print("    旧換算 /32768 側が 1.0 に近い（=表示値が約2.0）なら revert が必要です。")
        print("    静止させたら Enter を押してください（ノルム統計をリセットして2秒間の平均を取ります）")
        await ask_enter("")
        verifier.quat_norm_sum = 0.0
        verifier.quat_norm_count = 0
        await asyncio.sleep(2.0)
        stop_event.set()
        await monitor_task
        print_norm_judgement(verifier)

        stop_event = asyncio.Event()
        monitor_task = asyncio.create_task(live_monitor(verifier, stop_event))
        print("\n[2] 回転テスト: 机に平らに置いたまま、垂直軸まわりに"
              f"{REFERENCE_ANGLE_DEG:.0f}度だけ回します。")
        print("    Enter を押すと計測を開始します")
        await ask_enter("")
        verifier.start_recording()
        print(f"計測中... {REFERENCE_ANGLE_DEG:.0f}度回してください"
              "（1〜3秒くらいでゆっくり）。回し終わったら Enter を押してください")
        await ask_enter("")
        verifier.stop_recording()
        stop_event.set()
        await monitor_task
        print_rotation_result(verifier)

    finally:
        stop_event.set()
        if not monitor_task.done():
            monitor_task.cancel()
            try:
                await monitor_task
            except asyncio.CancelledError:
                pass
        if orphe.is_connected():
            print("\nStopping notification...")
            await orphe.stop_sensor_values_notification()
            await orphe.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    main_task = loop.create_task(main())

    try:
        loop.run_until_complete(main_task)
    except KeyboardInterrupt:
        print("KeyboardInterrupt(Ctrl+C) received. Canceling the main task...")
        main_task.cancel()
        try:
            loop.run_until_complete(main_task)
        except asyncio.CancelledError:
            pass
    finally:
        loop.close()
        print("Event loop closed.")
