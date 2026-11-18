"""Pure, offline-testable hold policy and planar self-weight model.

Angles refer to the physical arm, not the illustrative WebGL link lengths.
No serial I/O, auto-calibration, or human-arm load assumption.
"""
from __future__ import annotations

import copy
import math

DEFAULT_MOTION_POLICY = {
    "arrival_tolerance_deg": 1.5,
    "arrival_speed_rad_s": 0.03,
    "arrival_dwell_s": 0.5,
    "stall_error_deg": 3.0,
    "stall_speed_rad_s": 0.02,
    "stall_torque_ratio": 0.9,
    "stall_timeout_s": 2.0,
    "gravity_ramp_s": 1.0,
}

DEFAULT_GRAVITY = {
    "enabled": False,
    "calibrated": False,
    "shoulder_angle_sign": 1,
    "elbow_angle_sign": 1,
    "shoulder_offset_deg": 0.0,
    "elbow_offset_deg": 0.0,
    # CAD COM + user print masses + 317 g distal motor (2026-10-10).
    "shoulder_fixed_moment_nm": 1.39872366,
    "forearm_moment_nm": 0.05592679,
    "calibration_zeros_rad": {},
    "calibration_directions": {},
}

# Legacy opt-in residual compensation; migration keeps its stationary strategy.
DEFAULT_LOAD_COMPENSATION = {
    "enabled": False,
    "strategy": "stationary_residual",
    "ki_nm_per_rad_s": 0.6,
    "maximum_bias_nm": 0.5,
    "maximum_rate_nm_s": 0.1,
    "deadband_deg": 0.2,
    "maximum_error_deg": 5.0,
    "tracking_error_rate_rad_s": 0.12,
    "position_kp": 3.0,
    "velocity_kd": 1.0,
}

# Explicitly selected, experimental software preset. Never enabled by migration.
# These gains are not an identified motor/arm model or wearable validation.
SUPPORT_PRESET = {
    **DEFAULT_LOAD_COMPENSATION,
    "strategy": "tracking_support",
    "ki_nm_per_rad_s": 2.0,
    "maximum_bias_nm": 2.5,
    "maximum_rate_nm_s": 0.4,
    "maximum_error_deg": 15.0,
    "position_kp": 12.0,
    "velocity_kd": 1.5,
}


def motion_gains(config: dict, key: str) -> tuple[float, float]:
    load = load_compensation_config(config)
    if load["enabled"] and load["strategy"] == "tracking_support":
        return load["position_kp"], load["velocity_kd"]
    joint = config["joints"][key]
    return joint["motion_kp"], joint["motion_kd"]


def load_compensation_config(config: dict) -> dict:
    return {**DEFAULT_LOAD_COMPENSATION, **config.get("load_compensation", {})}


def residual_bias(settings: dict, bias: float, error_rad: float, base_torque: float,
                  torque_limit: float, dt: float, *, stationary: bool,
                  trajectory_complete: bool, tracking_error_rate_rad_s: float | None = None) -> float:
    """Bounded, rate-limited integral in motor coordinates with anti-windup.

    Legacy strategy learns only after settling and decays on retarget.
    Tracking support uses CURRENT trajectory error and independent encoder
    tracking-error rate, including during motion; retarget keeps its bias.
    Neither strategy learns outside its error window. A steady external push
    cannot be distinguished from gravity. Motor torque limiting still needs
    physical validation; this function only bounds its own feedforward bias.
    """
    if not settings["enabled"]:
        return 0.0
    cap = min(settings["maximum_bias_nm"], torque_limit)
    bias = max(-cap, min(cap, bias))
    step = settings["maximum_rate_nm_s"] * max(0.0, min(dt, 0.05))
    tracking = settings.get("strategy", "stationary_residual") == "tracking_support"
    if not tracking and not trajectory_complete:
        return bias - max(-step, min(step, bias))
    error_deg = abs(math.degrees(error_rad))
    learning_allowed = (tracking_error_rate_rad_s is not None
                        and math.isfinite(tracking_error_rate_rad_s)
                        and abs(tracking_error_rate_rad_s) <= settings["tracking_error_rate_rad_s"]) if tracking else stationary
    if not learning_allowed or not settings["deadband_deg"] < error_deg <= settings["maximum_error_deg"]:
        return bias
    delta = max(-step, min(step, settings["ki_nm_per_rad_s"] * error_rad * max(0.0, min(dt, 0.05))))
    total = base_torque + bias
    # Reject increments into saturation, but allow increments that unload it.
    if (total + delta > torque_limit and delta > 0) or (total + delta < -torque_limit and delta < 0):
        return bias
    return max(-cap, min(cap, bias + delta))


def policy(config: dict) -> dict:
    settings = {**DEFAULT_MOTION_POLICY, **config.get("motion_policy", {})}
    # Obsolete fields in schema-v2 files are ignored, without rewriting user settings.
    settings.pop("hold_error_deg", None)
    settings.pop("hold_error_timeout_s", None)
    settings.pop("convergence_timeout_s", None)
    return settings


def gravity_config(config: dict) -> dict:
    return {**copy.deepcopy(DEFAULT_GRAVITY), **config.get("gravity", {})}


def calibration_matches(config: dict) -> bool:
    gravity = gravity_config(config)
    return bool(gravity["calibrated"]) and all(
        gravity["calibration_zeros_rad"].get(key) is not None
        and gravity["calibration_zeros_rad"].get(key) == joint.get("software_zero_rad")
        and gravity["calibration_directions"].get(key) == joint["direction"]
        for key, joint in config["joints"].items()
    )


def validate_motion_settings(config: dict) -> None:
    settings = policy(config)
    ranges = {
        "arrival_tolerance_deg": (0.1, 5), "arrival_speed_rad_s": (0.005, 0.1),
        "arrival_dwell_s": (0.2, 2),
        "stall_error_deg": (1, 15), "stall_speed_rad_s": (0.005, 0.1),
        "stall_torque_ratio": (0.5, 1), "stall_timeout_s": (0.2, 5),
        "gravity_ramp_s": (0.5, 5),
    }
    if set(settings) != set(ranges):
        raise ValueError("运动保持配置字段无效")
    for name, (minimum, maximum) in ranges.items():
        value = float(settings[name])
        if not math.isfinite(value) or not minimum <= value <= maximum:
            raise ValueError(f"运动保持配置 {name} 超出安全范围")
    load = load_compensation_config(config)
    if set(load) != set(DEFAULT_LOAD_COMPENSATION) or not isinstance(load["enabled"], bool):
        raise ValueError("负载误差补偿字段或开关无效")
    if load["strategy"] not in ("stationary_residual", "tracking_support"):
        raise ValueError("负载控制策略无效")
    for name, bounds in {
        "ki_nm_per_rad_s": (0.01, 2.0), "maximum_bias_nm": (0.05, 2.5),
        "maximum_rate_nm_s": (0.01, 0.4), "deadband_deg": (0.05, 1.0),
        "maximum_error_deg": (1.0, 15.0), "tracking_error_rate_rad_s": (0.02, 0.15),
        "position_kp": (0.1, 20.0), "velocity_kd": (0.1, 5.0),
    }.items():
        value = load[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not bounds[0] <= value <= bounds[1]:
            raise ValueError(f"负载误差补偿 {name} 超出范围")
    if load["enabled"] and load["deadband_deg"] >= settings["arrival_tolerance_deg"]:
        raise ValueError("补偿死区必须小于到位容差")
    gravity = gravity_config(config)
    if set(gravity) != set(DEFAULT_GRAVITY):
        raise ValueError("重力补偿配置字段无效")
    if not isinstance(gravity["enabled"], bool) or not isinstance(gravity["calibrated"], bool):
        raise ValueError("补偿开关和标定状态必须为布尔值")
    for name in ("shoulder_angle_sign", "elbow_angle_sign"):
        if gravity[name] not in (-1, 1):
            raise ValueError("物理角度正方向只能是 +1 或 -1")
    for name in ("shoulder_offset_deg", "elbow_offset_deg"):
        if not math.isfinite(float(gravity[name])) or abs(float(gravity[name])) > 720:
            raise ValueError("重力补偿角度偏置无效")
    for name, maximum in (("shoulder_fixed_moment_nm", 2.0), ("forearm_moment_nm", 0.3)):
        if not math.isfinite(float(gravity[name])) or not 0 <= float(gravity[name]) <= maximum:
            raise ValueError("空载补偿系数超出范围，增加人臂负载前须重新核算")
    if gravity["enabled"] and not calibration_matches(config):
        raise ValueError("重力补偿未标定或零位/方向已变化，禁止开启")


def gravity_torques(config: dict, angles_deg: dict[str, float]) -> dict[str, float]:
    """Return software-coordinate torques; caller converts to raw motor sign.

    theta_s is absolute upper-arm angle, theta_f absolute forearm angle,
    measured CCW from the horizontal in the physical lifting plane.
    Joint signs map software coordinates to these physical angles. The model
    assumes parallel horizontal axes; mounting tilt needs a different model.
    """
    gravity = gravity_config(config)
    if not gravity["enabled"]:
        return {"shoulder": 0.0, "elbow": 0.0}
    if not calibration_matches(config):
        raise RuntimeError("重力补偿标定已失效")
    if set(angles_deg) != {"shoulder", "elbow"} or not all(math.isfinite(v) for v in angles_deg.values()):
        raise RuntimeError("重力补偿缺少两关节真实角度")
    s, e = gravity["shoulder_angle_sign"], gravity["elbow_angle_sign"]
    theta_s = math.radians(s * angles_deg["shoulder"] + gravity["shoulder_offset_deg"])
    theta_f = theta_s + math.radians(e * angles_deg["elbow"] + gravity["elbow_offset_deg"])
    forearm = gravity["forearm_moment_nm"] * math.cos(theta_f)
    return {
        "shoulder": s * (gravity["shoulder_fixed_moment_nm"] * math.cos(theta_s) + forearm),
        "elbow": e * forearm,
    }
