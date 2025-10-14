import asyncio
import matplotlib.pyplot as plt
from collections import deque
from orphe_insole import Orphe
import matplotlib.animation as animation

plot_buffer_size = 512  # バッファサイズ
update_interval = 0.02  # 描画更新間隔（秒）

# データ配列（固定長デック）
sensors = [deque(maxlen=plot_buffer_size) for _ in range(6)]


# プロットの初期設定
fig, ax = plt.subplots()
lines = [ax.plot([], [], label=f'Sensor {i+1}')[0] for i in range(6)]
ax.legend()
ax.set_xlim(0, plot_buffer_size)
ax.set_ylim(0, 1024)  # 適宜調整

# アニメーション更新関数


def update(frame):
    # すでに更新されている acc データをプロット
    for i, line in enumerate(lines):
        line.set_data(range(len(sensors[i])), sensors[i])
    return lines


async def main():

    def got_pressure(pressure):
        print(f"Pressure: {[f'{p:.2f}' for p in pressure.values]}")
        for i, p in enumerate(pressure.values):
            sensors[i].append(p)

    def on_disconnect():
        print("Disconnected from the ORPHE CORE device.")

    def lost_data(serial_number_prev, serial_number):
        print(f"Data loss detected. {serial_number_prev} <-> {serial_number}")

    orphe = Orphe()
    orphe.set_got_pressure_callback(got_pressure)
    orphe.set_lost_data_callback(lost_data)

    if not await orphe.connect('5DA2B599-7083-AD42-5A6D-985CBC95F122'):
        return
    # if not await orphe.connect('10BDC1A0-2C23-8B6C-9F69-FFDF5CFC2891'):
    #     return

    di = await orphe.read_device_information()
    await orphe.set_data_streaming_mode(4)
    await orphe.start_sensor_values_notification()

    # アニメーション設定（frames指定を省略して無限更新）
    ani = animation.FuncAnimation(fig, update, interval=20, blit=True)

    # ブロッキングせず（他のコールバック関数処理を止めないため）にウィンドウを表示
    plt.show(block=False)

    # BLE接続中は定期的にGUIイベント処理を行う
    try:
        while orphe.is_connected():
            plt.pause(update_interval)  # GUI更新
            await asyncio.sleep(update_interval)
    finally:
        if orphe.is_connected():
            print("Stopping notification...")
            await orphe.stop_sensor_values_notification()
            print("Notification stopped. Disconnecting from the device.")
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
