#!/usr/bin/env python3
"""Bleak host for can_sniffer_c3 (Nordic UART / NUS).

The C3 advertises as C3-CAN. This is a laptop console, not ble-serial:
frames print here, commands you type are written to the RX characteristic.

  python3 -m pip install -r host/requirements.txt
  python3 host/sniffer.py
  python3 host/sniffer.py --name C3-CAN --log /tmp/can.log

Phone: Serial Bluetooth Terminal, BLE profile Nordic UART — do not connect
the phone and this script at the same time.
"""

from __future__ import annotations

import argparse
import asyncio
import sys
from datetime import datetime, timezone

from bleak import BleakClient, BleakScanner

NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write: laptop -> C3
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify: C3 -> laptop


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def _rssi(dev) -> int:
    r = getattr(dev, "rssi", None)
    if r is not None:
        return int(r)
    md = getattr(dev, "metadata", None)
    if isinstance(md, dict) and md.get("rssi") is not None:
        return int(md["rssi"])
    return -100


async def find_device(name: str, address: str | None, timeout: float):
    if address:
        return await BleakScanner.find_device_by_address(address, timeout=timeout)
    by_name = getattr(BleakScanner, "find_device_by_name", None)
    if callable(by_name):
        return await by_name(name, timeout=timeout)
    found = None
    best = -999
    devices = await BleakScanner.discover(timeout=timeout)
    for d in devices:
        if d.name == name:
            rssi = _rssi(d)
            if rssi > best:
                best = rssi
                found = d
    return found


async def stdin_lines():
    loop = asyncio.get_event_loop()
    while True:
        line = await loop.run_in_executor(None, sys.stdin.readline)
        if line == "":
            return
        yield line.rstrip("\r\n")


async def run(args: argparse.Namespace) -> int:
    log(f"scanning for {args.address or args.name} …")
    dev = await find_device(args.name, args.address, args.timeout)
    if dev is None:
        log("not found. Is the C3 advertising, and is another phone already connected?")
        return 1
    log(f"connecting {dev.name}  {dev.address}")

    log_fp = open(args.log, "a", encoding="utf-8") if args.log else None
    buf = bytearray()

    def on_tx(_handle: int, data: bytearray) -> None:
        nonlocal buf
        buf.extend(data)
        while True:
            nl = buf.find(b"\n")
            if nl < 0:
                break
            line = buf[:nl].decode("utf-8", errors="replace").rstrip("\r")
            del buf[: nl + 1]
            print(line, flush=True)
            if log_fp:
                ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
                log_fp.write(f"{ts} {line}\n")
                log_fp.flush()

    async with BleakClient(dev, timeout=args.timeout) as client:
        await client.start_notify(NUS_TX, on_tx)
        log("subscribed. type commands (all, 0x2E1, 500, s, d, r, st, h). Ctrl-C to quit.")
        try:
            async for line in stdin_lines():
                if not line:
                    continue
                payload = (line + "\n").encode("utf-8")
                await client.write_gatt_char(NUS_RX, payload, response=False)
        except (asyncio.CancelledError, KeyboardInterrupt):
            pass
        finally:
            try:
                await client.stop_notify(NUS_TX)
            except Exception:
                pass

    if log_fp:
        log_fp.close()
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description="C3-CAN Bleak sniffer host")
    p.add_argument("--name", default="C3-CAN", help="BLE advertised name")
    p.add_argument("--address", help="connect this BLE address, skip name scan")
    p.add_argument("--timeout", type=float, default=10.0)
    p.add_argument("--log", help="append decoded lines to this file")
    args = p.parse_args()
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
