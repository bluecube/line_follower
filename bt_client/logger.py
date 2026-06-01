#!/bin/env python3
import asyncio
import sys
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "LineFollower"
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


def on_notify(_, data: bytearray) -> None:
    print(data.decode("utf-8", errors="replace"), end="", flush=True)


async def main() -> None:
    print(f"Scanning for '{DEVICE_NAME}'...", file=sys.stderr)
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10)
    if device is None:
        print(f"Device '{DEVICE_NAME}' not found.", file=sys.stderr)
        sys.exit(1)

    print(f"Found {device.address}, connecting...", file=sys.stderr)
    async with BleakClient(device) as client:
        print("Connected. Streaming log (Ctrl+C to stop).\n", file=sys.stderr)
        await client.start_notify(NUS_TX, on_notify)
        try:
            await asyncio.Event().wait()
        except asyncio.CancelledError:
            pass


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
