"""Safely probe a CyberGear and change CAN ID 127 to 126 over SLCAN."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))

import serial

from cybergear_protocol import (
    HOST_ID_DEFAULT,
    decode_slcan_extended,
    encode_slcan_extended,
    make_get_device_id,
    make_set_can_id,
)


def send(device: serial.Serial, frame: tuple[int, bytes]) -> None:
    device.write(encode_slcan_extended(*frame))
    device.flush()


def probe(device: serial.Serial, motor_id: int, duration: float = 0.45) -> set[tuple[int, str]]:
    device.reset_input_buffer()
    send(device, make_get_device_id(motor_id, HOST_ID_DEFAULT))
    deadline = time.monotonic() + duration
    found: set[tuple[int, str]] = set()
    while time.monotonic() < deadline:
        raw = device.read_until(b"\r", 80)
        if not raw:
            continue
        decoded = decode_slcan_extended(raw.decode("ascii", errors="ignore"))
        if not decoded:
            continue
        can_id, payload = decoded
        if ((can_id >> 24) & 0x1F) == 0 and len(payload) == 8:
            found.add(((can_id >> 8) & 0xFF, payload.hex().upper()))
    return found


def scan_all(device: serial.Serial) -> set[tuple[int, str]]:
    """Broadcast addressed read-only UID requests across the valid ID range."""
    device.reset_input_buffer()
    for motor_id in range(128):
        send(device, make_get_device_id(motor_id, HOST_ID_DEFAULT))
        time.sleep(0.006)
    deadline = time.monotonic() + 1.2
    found: set[tuple[int, str]] = set()
    while time.monotonic() < deadline:
        raw = device.read_until(b"\r", 80)
        if not raw:
            continue
        decoded = decode_slcan_extended(raw.decode("ascii", errors="ignore"))
        if not decoded:
            continue
        can_id, payload = decoded
        if ((can_id >> 24) & 0x1F) == 0 and len(payload) == 8:
            found.add(((can_id >> 8) & 0xFF, payload.hex().upper()))
    return found


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM11")
    parser.add_argument("--old", type=int, default=127)
    parser.add_argument("--new", type=int, default=126)
    args = parser.parse_args()

    with serial.Serial(args.port, 921600, timeout=0.04, write_timeout=0.5) as device:
        for command in (b"C\r", b"S8\r", b"O\r"):
            device.write(command)
            device.flush()
            time.sleep(0.08)

        old_before = probe(device, args.old)
        new_before = probe(device, args.new)
        print(f"BEFORE old={sorted(old_before)} new={sorted(new_before)}")

        if new_before and old_before:
            print("ABORT: both IDs are already present")
            return 3
        if new_before and not old_before:
            print("UNCHANGED: motor is already using the requested new ID")
            return 0
        if not old_before:
            print(f"SCAN all={sorted(scan_all(device))}")
            print("ABORT: old ID did not respond")
            return 4

        send(device, make_set_can_id(args.old, args.new, HOST_ID_DEFAULT))
        time.sleep(0.35)
        new_after = probe(device, args.new, 0.7)
        old_after = probe(device, args.old, 0.35)
        print(f"AFTER old={sorted(old_after)} new={sorted(new_after)}")
        if not new_after or old_after:
            print("FAILED: ID change could not be verified")
            return 5
        print("SUCCESS: CAN ID change verified")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
