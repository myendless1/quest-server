from __future__ import annotations

from dataclasses import dataclass, field
import math
import threading
import time
from typing import Any, Callable, Dict, Iterable, Optional, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def _v_add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _q_normalize(q: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def _q_conjugate(q: Quaternion) -> Quaternion:
    return (-q[0], -q[1], -q[2], q[3])


def _q_multiply_raw(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _q_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    return _q_normalize(_q_multiply_raw(a, b))


def _q_inverse(q: Quaternion) -> Quaternion:
    return _q_conjugate(_q_normalize(q))


def _q_rotate(q: Quaternion, v: Vector3) -> Vector3:
    qv = (v[0], v[1], v[2], 0.0)
    q_norm = _q_normalize(q)
    rotated = _q_multiply_raw(_q_multiply_raw(q_norm, qv), _q_inverse(q_norm))
    return (rotated[0], rotated[1], rotated[2])


def _q_to_rotvec(q: Quaternion) -> Vector3:
    x, y, z, w = _q_normalize(q)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    w = max(-1.0, min(1.0, w))
    angle = 2.0 * math.acos(w)
    sin_half = math.sqrt(max(0.0, 1.0 - w * w))
    if sin_half < 1e-8:
        return (2.0 * x, 2.0 * y, 2.0 * z)
    axis = (x / sin_half, y / sin_half, z / sin_half)
    return (axis[0] * angle, axis[1] * angle, axis[2] * angle)


def _now() -> float:
    return time.time()


def _as_list(v: Iterable[float]) -> list[float]:
    return [float(x) for x in v]


@dataclass(frozen=True)
class Pose:
    position: Vector3
    orientation: Quaternion
    stamp: float = field(default_factory=_now)
    frame_id: str = ""

    @classmethod
    def from_ros(cls, msg: Any) -> "Pose":
        pose_msg = getattr(msg, "pose", msg)
        position = getattr(pose_msg, "position")
        orientation = getattr(pose_msg, "orientation")
        return cls(
            position=(float(position.x), float(position.y), float(position.z)),
            orientation=_q_normalize(
                (
                    float(orientation.x),
                    float(orientation.y),
                    float(orientation.z),
                    float(orientation.w),
                )
            ),
            stamp=ros_time_to_float(getattr(getattr(msg, "header", None), "stamp", None)),
            frame_id=str(getattr(getattr(msg, "header", None), "frame_id", "")),
        )


@dataclass(frozen=True)
class Twist:
    linear: Vector3
    angular: Vector3
    stamp: float = field(default_factory=_now)

    @classmethod
    def from_ros(cls, msg: Any) -> "Twist":
        twist_msg = getattr(msg, "twist", msg)
        linear = getattr(twist_msg, "linear")
        angular = getattr(twist_msg, "angular")
        return cls(
            linear=(float(linear.x), float(linear.y), float(linear.z)),
            angular=(float(angular.x), float(angular.y), float(angular.z)),
            stamp=ros_time_to_float(getattr(getattr(msg, "header", None), "stamp", None)),
        )


def ros_time_to_float(stamp: Any) -> float:
    if stamp is None:
        return _now()
    if hasattr(stamp, "to_sec"):
        return float(stamp.to_sec())
    if hasattr(stamp, "nanoseconds"):
        return float(stamp.nanoseconds) / 1e9
    sec = float(getattr(stamp, "sec", getattr(stamp, "secs", 0.0)))
    nsec = float(getattr(stamp, "nanosec", getattr(stamp, "nsecs", 0.0)))
    if sec == 0.0 and nsec == 0.0:
        return _now()
    return sec + nsec / 1e9


def extract_trigger_value(msg: Any, field_name: str = "press_index") -> float:
    value = getattr(msg, field_name, 0.0)
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    return float(value)


@dataclass
class HandMotionState:
    hand: str
    trigger_threshold: float = 0.5
    trigger_field: str = "press_index"
    latest_pose: Optional[Pose] = None
    latest_twist: Optional[Twist] = None
    latest_input: Optional[Any] = None
    trigger_value: float = 0.0
    active: bool = False
    start_pose: Optional[Pose] = None
    start_time: Optional[float] = None
    last_update_time: Optional[float] = None
    pressed_edge: bool = False
    released_edge: bool = False

    def update_pose(self, pose: Pose) -> None:
        self.latest_pose = pose
        self.last_update_time = pose.stamp
        if self.active and self.start_pose is None:
            self.start_pose = pose
            self.start_time = pose.stamp

    def update_twist(self, twist: Twist) -> None:
        self.latest_twist = twist
        self.last_update_time = twist.stamp

    def update_input(self, msg: Any) -> None:
        previous = self.active
        self.latest_input = msg
        self.trigger_value = extract_trigger_value(msg, self.trigger_field)
        self.active = self.trigger_value >= self.trigger_threshold
        self.pressed_edge = self.active and not previous
        self.released_edge = (not self.active) and previous
        self.last_update_time = _now()

        if self.pressed_edge:
            self.start_pose = self.latest_pose
            self.start_time = self.latest_pose.stamp if self.latest_pose else self.last_update_time
        elif self.released_edge:
            self.start_pose = None
            self.start_time = None

    def snapshot(self) -> Dict[str, Any]:
        delta_pos_base: Vector3 = (0.0, 0.0, 0.0)
        delta_pos_local: Vector3 = (0.0, 0.0, 0.0)
        delta_quat_local: Quaternion = (0.0, 0.0, 0.0, 1.0)

        if self.active and self.latest_pose and self.start_pose:
            delta_pos_base = _v_sub(self.latest_pose.position, self.start_pose.position)
            start_inv = _q_inverse(self.start_pose.orientation)
            delta_pos_local = _q_rotate(start_inv, delta_pos_base)
            delta_quat_local = _q_multiply(start_inv, self.latest_pose.orientation)

        return {
            "hand": self.hand,
            "active": self.active,
            "pressed_edge": self.pressed_edge,
            "released_edge": self.released_edge,
            "trigger": self.trigger_value,
            "start_time": self.start_time,
            "last_update_time": self.last_update_time,
            "pose": pose_to_dict(self.latest_pose),
            "twist": twist_to_dict(self.latest_twist),
            "delta_pos_base": _as_list(delta_pos_base),
            "delta_pos_local": _as_list(delta_pos_local),
            "delta_quat_local": _as_list(delta_quat_local),
            "delta_rotvec_local": _as_list(_q_to_rotvec(delta_quat_local)),
        }


def pose_to_dict(pose: Optional[Pose]) -> Optional[Dict[str, Any]]:
    if pose is None:
        return None
    return {
        "position": _as_list(pose.position),
        "orientation": _as_list(pose.orientation),
        "stamp": pose.stamp,
        "frame_id": pose.frame_id,
    }


def twist_to_dict(twist: Optional[Twist]) -> Optional[Dict[str, Any]]:
    if twist is None:
        return None
    return {
        "linear": _as_list(twist.linear),
        "angular": _as_list(twist.angular),
        "stamp": twist.stamp,
    }


class QuestMotionServerState:
    def __init__(
        self,
        trigger_threshold: float = 0.5,
        trigger_field: str = "press_index",
        clock: Callable[[], float] = _now,
    ) -> None:
        self._clock = clock
        self._lock = threading.RLock()
        self.left = HandMotionState("left", trigger_threshold, trigger_field)
        self.right = HandMotionState("right", trigger_threshold, trigger_field)

    def update_pose(self, hand: str, msg_or_pose: Any) -> None:
        pose = msg_or_pose if isinstance(msg_or_pose, Pose) else Pose.from_ros(msg_or_pose)
        with self._lock:
            self._hand(hand).update_pose(pose)

    def update_twist(self, hand: str, msg_or_twist: Any) -> None:
        twist = msg_or_twist if isinstance(msg_or_twist, Twist) else Twist.from_ros(msg_or_twist)
        with self._lock:
            self._hand(hand).update_twist(twist)

    def update_input(self, hand: str, msg: Any) -> None:
        with self._lock:
            self._hand(hand).update_input(msg)

    def get_left_delta(self) -> Dict[str, Any]:
        return self.get_latest()["left"]

    def get_right_delta(self) -> Dict[str, Any]:
        return self.get_latest()["right"]

    def get_active_hand(self) -> Optional[str]:
        snapshot = self.get_latest()
        active = snapshot["active_hands"]
        if len(active) == 1:
            return active[0]
        return None

    def get_latest(self) -> Dict[str, Any]:
        with self._lock:
            left = self.left.snapshot()
            right = self.right.snapshot()
            active_hands = [hand for hand, state in (("left", left), ("right", right)) if state["active"]]
            return {
                "timestamp": self._clock(),
                "active_hand": active_hands[0] if len(active_hands) == 1 else None,
                "active_hands": active_hands,
                "left": left,
                "right": right,
            }

    def _hand(self, hand: str) -> HandMotionState:
        if hand == "left":
            return self.left
        if hand == "right":
            return self.right
        raise ValueError(f"unknown hand: {hand!r}")
