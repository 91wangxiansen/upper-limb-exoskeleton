"""CyberGear CAN 2.0B protocol helpers.

Frame layout and command types follow the supplied CyberGear manual, chapter 4.
All IDs are 29-bit extended CAN identifiers and every payload is eight bytes.
"""

from __future__ import annotations

import struct
import math
from dataclasses import dataclass


HOST_ID_DEFAULT = 0xFD

CMD_GET_DEVICE_ID = 0
CMD_MOTOR_CONTROL = 1
CMD_FEEDBACK = 2
CMD_ENABLE = 3
CMD_STOP = 4
CMD_SET_ZERO = 6
CMD_SET_CAN_ID = 7
CMD_PARAM_READ = 17
CMD_PARAM_WRITE = 18
CMD_FAULT_FEEDBACK = 21

RUN_MODE_CONTROL = 0
RUN_MODE_POSITION = 1
RUN_MODE_SPEED = 2
RUN_MODE_CURRENT = 3

PARAM_RUN_MODE = 0x7005
PARAM_IQ_REF = 0x7006
PARAM_SPD_REF = 0x700A
PARAM_LIMIT_TORQUE = 0x700B
PARAM_LOC_REF = 0x7016
PARAM_LIMIT_SPD = 0x7017
PARAM_LIMIT_CUR = 0x7018
PARAM_MECH_POS = 0x7019
PARAM_IQF = 0x701A
PARAM_MECH_VEL = 0x701B
PARAM_VBUS = 0x701C
PARAM_LOC_KP = 0x701E
PARAM_SPD_KP = 0x701F
PARAM_SPD_KI = 0x7020


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def float_to_uint(value: float, low: float, high: float, bits: int = 16) -> int:
    value = clamp(value, low, high)
    return int((value - low) * ((1 << bits) - 1) / (high - low))


def uint_to_float(value: int, low: float, high: float, bits: int = 16) -> float:
    return value * (high - low) / ((1 << bits) - 1) + low


def command_id(command: int, motor_id: int, host_id: int = HOST_ID_DEFAULT) -> int:
    """ID format for command types 0, 3, 4, 6, 7, 17, 18 and 21."""
    return ((command & 0x1F) << 24) | ((host_id & 0xFFFF) << 8) | (motor_id & 0xFF)


def encode_slcan_extended(can_id: int, payload: bytes) -> bytes:
    if not 0 <= can_id <= 0x1FFFFFFF:
        raise ValueError("CAN ID must be a 29-bit extended identifier")
    if len(payload) > 8:
        raise ValueError("CyberGear payload must be at most 8 bytes")
    return f"T{can_id:08X}{len(payload):X}{payload.hex().upper()}\r".encode("ascii")


def decode_slcan_extended(line: str) -> tuple[int, bytes] | None:
    line = line.strip()
    if not line.startswith("T") or len(line) < 10:
        return None
    try:
        can_id = int(line[1:9], 16)
        dlc = int(line[9], 16)
        raw = bytes.fromhex(line[10:10 + dlc * 2])
    except ValueError:
        return None
    return (can_id, raw) if len(raw) == dlc else None


def payload_param(index: int, value: float | int | None = None, value_type: str = "float") -> bytes:
    data = bytearray(8)
    data[0:2] = struct.pack("<H", index)
    if value is not None:
        if value_type == "u8":
            data[4] = int(value) & 0xFF
        elif value_type == "float":
            data[4:8] = struct.pack("<f", float(value))
        else:
            raise ValueError(f"unsupported parameter type: {value_type}")
    return bytes(data)


def make_param_read(motor_id: int, index: int, host_id: int = HOST_ID_DEFAULT) -> tuple[int, bytes]:
    return command_id(CMD_PARAM_READ, motor_id, host_id), payload_param(index)


def make_param_write(
    motor_id: int,
    index: int,
    value: float | int,
    value_type: str = "float",
    host_id: int = HOST_ID_DEFAULT,
) -> tuple[int, bytes]:
    return command_id(CMD_PARAM_WRITE, motor_id, host_id), payload_param(index, value, value_type)


def make_enable(motor_id: int, host_id: int = HOST_ID_DEFAULT) -> tuple[int, bytes]:
    return command_id(CMD_ENABLE, motor_id, host_id), bytes(8)


def make_stop(motor_id: int, host_id: int = HOST_ID_DEFAULT) -> tuple[int, bytes]:
    return command_id(CMD_STOP, motor_id, host_id), bytes(8)


def make_get_device_id(motor_id: int, host_id: int = HOST_ID_DEFAULT) -> tuple[int, bytes]:
    return command_id(CMD_GET_DEVICE_ID, motor_id, host_id), bytes(8)


def make_set_can_id(
    old_motor_id: int,
    new_motor_id: int,
    host_id: int = HOST_ID_DEFAULT,
) -> tuple[int, bytes]:
    """Manual communication type 7: change ``old_motor_id`` to ``new_motor_id``."""
    if not 0 <= old_motor_id <= 127 or not 0 <= new_motor_id <= 127:
        raise ValueError("CyberGear CAN ID must be in the range 0..127")
    # Type 7 layout from manual section 4.1.7:
    # bit23..16 = new motor ID, bit15..8 = host ID, bit7..0 = old motor ID.
    can_id = (
        ((CMD_SET_CAN_ID & 0x1F) << 24)
        | ((new_motor_id & 0xFF) << 16)
        | ((host_id & 0xFF) << 8)
        | old_motor_id
    )
    return can_id, bytes(8)


def make_motor_control(
    motor_id: int,
    torque_nm: float,
    position_rad: float,
    speed_rad_s: float,
    kp: float,
    kd: float,
) -> tuple[int, bytes]:
    """Manual section 4.1.2 operation-control frame (type 1)."""
    torque = float_to_uint(torque_nm, -12.0, 12.0)
    can_id = ((CMD_MOTOR_CONTROL & 0x1F) << 24) | ((torque & 0xFFFF) << 8) | (motor_id & 0xFF)
    values = (
        float_to_uint(position_rad, -4.0 * 3.141592653589793, 4.0 * 3.141592653589793),
        float_to_uint(speed_rad_s, -30.0, 30.0),
        float_to_uint(kp, 0.0, 500.0),
        float_to_uint(kd, 0.0, 5.0),
    )
    return can_id, struct.pack(">HHHH", *values)


@dataclass(frozen=True)
class Feedback:
    motor_id: int
    host_id: int
    mode_status: int
    fault_bits: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    temperature_c: float


def decode_feedback(can_id: int, payload: bytes) -> Feedback | None:
    if ((can_id >> 24) & 0x1F) != CMD_FEEDBACK or len(payload) != 8:
        return None
    status = (can_id >> 8) & 0xFFFF
    motor_id = status & 0xFF
    fault_bits = (status >> 8) & 0x3F
    mode_status = (status >> 14) & 0x03
    pos_u, vel_u, torque_u, temp_u = struct.unpack(">HHHH", payload)
    return Feedback(
        motor_id=motor_id,
        host_id=can_id & 0xFF,
        mode_status=mode_status,
        fault_bits=fault_bits,
        position_rad=uint_to_float(pos_u, -4.0 * 3.141592653589793, 4.0 * 3.141592653589793),
        velocity_rad_s=uint_to_float(vel_u, -30.0, 30.0),
        torque_nm=uint_to_float(torque_u, -12.0, 12.0),
        temperature_c=temp_u / 10.0,
    )


def decode_param_value(payload: bytes, value_type: str = "float") -> tuple[int, float | int] | None:
    if len(payload) != 8:
        return None
    index = struct.unpack("<H", payload[0:2])[0]
    if value_type == "u8":
        return index, payload[4]
    if value_type == "float":
        return index, struct.unpack("<f", payload[4:8])[0]
    return None


PARAMETER_DIAGNOSTICS = {
    PARAM_RUN_MODE: ("run_mode", "u8", 0, 3),
    PARAM_LIMIT_TORQUE: ("limit_torque_nm", "float", 0, 12),
    PARAM_LIMIT_CUR: ("limit_cur_a", "float", 0, 23),
    PARAM_IQF: ("iq_a", "float", -23, 23),
    PARAM_VBUS: ("vbus_v", "float", 0, 60),
}


def decode_diagnostic_parameter(can_id: int, payload: bytes, host_id: int,
                                motor_ids: set[int]) -> tuple[int, int, float | int] | None:
    """Decode only type-17, reserved/status-zero, finite known parameters.

    Address-swapped responses are used by CyberGear firmware; also recognize
    the manual's host-in-bit15..8 layout. Caller must correlate a pending
    read: an unsolicited frame must not verify a write or refresh freshness.
    """
    if (can_id >> 24) & 0x1F != CMD_PARAM_READ or len(payload) != 8:
        return None
    if can_id & 0xFF0000 or payload[2:4] != b"\x00\x00":
        return None
    upper, lower = (can_id >> 8) & 0xFF, can_id & 0xFF
    motor_id = upper if lower == host_id else lower if upper == host_id else None
    if motor_id not in motor_ids:
        return None
    index = struct.unpack("<H", payload[:2])[0]
    if index not in PARAMETER_DIAGNOSTICS:
        return None
    _name, kind, minimum, maximum = PARAMETER_DIAGNOSTICS[index]
    value = decode_param_value(payload, kind)[1]
    if not math.isfinite(value) or not minimum <= value <= maximum:
        return None
    return motor_id, index, value
