import asyncio
from orphe_insole import Orphe  # orphe_core.pyからOrpheクラスをインポート


async def main():
    orphe = Orphe()
    devices = await orphe.scan_all_devices()
    for address, (device, adv_data) in devices.items():
        print(
            f"Device: {device.name}(name), {device.address}(address), {adv_data.rssi}(rssi)")

if __name__ == "__main__":
    asyncio.run(main())
