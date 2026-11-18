"""Read the three volatile CyberGear cascade-controller gains without enabling it."""

from __future__ import annotations

import sys
import time
from pathlib import Path

import serial


APP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_ROOT))

from cybergear_protocol import (  # noqa: E402
    CMD_PARAM_READ,
    HOST_ID_DEFAULT,
    PARAM_LOC_KP,
    PARAM_SPD_KI,
    PARAM_SPD_KP,
    decode_param_value,
    decode_slcan_extended,
    encode_slcan_extended,
    make_param_read,
)


PARAMETERS = {
    PARAM_LOC_KP: "loc_kp",
    PARAM_SPD_KP: "spd_kp",
    PARAM_SPD_KI: "spd_ki",
}


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM11"
    motor_id = int(sys.argv[2]) if len(sys.argv) > 2 else 127
    values: dict[int, float] = {}
    errors: dict[int, int] = {}
    with serial.Serial(port, 921600, timeout=0.05, write_timeout=1) as device:
        for command in (b"C\r", b"S8\r", b"M0\r", b"A0\r", b"O\r"):
            device.write(command)
            device.flush()
            time.sleep(0.03)
        device.reset_input_buffer()
        for index in PARAMETERS:
            can_id, payload = make_param_read(motor_id, index, HOST_ID_DEFAULT)
            device.write(encode_slcan_extended(can_id, payload))
            device.flush()
            deadline = time.monotonic() + 0.7
            buffer = ""
            while time.monotonic() < deadline and index not in values and index not in errors:
                chunk = device.read(device.in_waiting or 1).decode("ascii", errors="ignore")
                if not chunk:
                    continue
                buffer += chunk
                while "\r" in buffer:
                    line, buffer = buffer.split("\r", 1)
                    decoded = decode_slcan_extended(line)
                    if not decoded:
                        continue
                    response_id, response_payload = decoded
                    if ((response_id >> 24) & 0x1F) != CMD_PARAM_READ:
                        continue
                    response_status = ((response_id >> 8) & 0xFFFF) >> 8
                    parameter = decode_param_value(response_payload, "float")
                    if parameter:
                        returned_index, value = parameter
                        if response_status:
                            errors[returned_index] = response_status
                            print(f"RX {line} -> index=0x{returned_index:04X} ERROR_STATUS=0x{response_status:02X}")
                        else:
                            print(f"RX {line} -> index=0x{returned_index:04X} data={response_payload[4:8].hex(' ').upper()}")
                            values[returned_index] = float(value)
            time.sleep(0.03)
        device.write(b"C\r")
        device.flush()

    for index, name in PARAMETERS.items():
        if index in values:
            print(f"{name}= {values[index]:.8g} (0x{index:04X})")
        elif index in errors:
            print(f"{name}= UNSUPPORTED_OR_ERROR_0x{errors[index]:02X} (0x{index:04X})")
        else:
            print(f"{name}= NO_RESPONSE (0x{index:04X})")
    return 0 if len(values) == len(PARAMETERS) else 2


if __name__ == "__main__":
    raise SystemExit(main())
