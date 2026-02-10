from __future__ import annotations

from dataclasses import dataclass
from collections import deque
from typing import Any
import time
import numpy as np
import yaml


@dataclass
class SafetyConfig:
    enable: bool = True
    dry_run: bool = False
    log_events: bool = True
    command_hold_frames: int = 3

    # action limits
    max_action_abs: float | None = 3.5
    max_action_delta: float | None = 0.25

    # gain limits
    max_kp: float | None = 300.0
    max_kd: float | None = 20.0
    max_kp_delta: float | None = 50.0
    max_kd_delta: float | None = 5.0

    # damping fallback
    damping_kd: float = 8.0

    # fault handling
    fault_hold_steps: int = 25
    fault_latch_seconds: float = 1.0
    max_faults_before_latch: int = 3


def _to_bool(v: Any, default: bool) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        return v.strip().lower() in {"1", "true", "yes", "y", "on"}
    return default


def load_safety_config(path: str | None, defaults: SafetyConfig | None = None) -> SafetyConfig:
    cfg = defaults if defaults is not None else SafetyConfig()
    if path is None:
        return cfg
    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        return cfg

    # shallow merge
    for k, v in data.items():
        if not hasattr(cfg, k):
            continue
        if isinstance(getattr(cfg, k), bool):
            setattr(cfg, k, _to_bool(v, getattr(cfg, k)))
        else:
            setattr(cfg, k, v)
    return cfg


class HoldToConfirm:
    """Require a command to be held for N frames before triggering once."""

    def __init__(self, hold_frames: int):
        self.hold_frames = max(1, int(hold_frames))
        self._count: dict[str, int] = {}
        self._latched: dict[str, bool] = {}

    def trigger(self, key: str, pressed: bool) -> bool:
        if not pressed:
            self._count[key] = 0
            self._latched[key] = False
            return False
        if self._latched.get(key, False):
            return False
        self._count[key] = self._count.get(key, 0) + 1
        if self._count[key] >= self.hold_frames:
            self._latched[key] = True
            return True
        return False


class SafetyFilter:
    def __init__(self, num_joints: int, cfg: SafetyConfig):
        self.num_joints = num_joints
        self.cfg = cfg
        self._prev_action = np.zeros(num_joints, dtype=np.float32)
        self._prev_kp = np.zeros(num_joints, dtype=np.float32)
        self._prev_kd = np.zeros(num_joints, dtype=np.float32)
        self._events = deque(maxlen=50)
        self._fault_count = 0
        self._fault_until_step = 0
        self._latch_until_time = 0.0
        self._step = 0

    def _log(self, msg: str):
        self._events.append((time.time(), msg))
        if self.cfg.log_events:
            print(f"[Safety] {msg}")

    def _fault(self, reason: str):
        self._fault_count += 1
        self._fault_until_step = max(self._fault_until_step, self._step + self.cfg.fault_hold_steps)
        if self._fault_count >= self.cfg.max_faults_before_latch:
            self._latch_until_time = time.time() + self.cfg.fault_latch_seconds
        self._log(f"fault: {reason} (count={self._fault_count})")

    def report_fault(self, reason: str):
        if self.cfg.enable:
            self._fault(reason)

    def recent_events(self):
        return list(self._events)

    def _should_force_damping(self) -> bool:
        if self._step < self._fault_until_step:
            return True
        if time.time() < self._latch_until_time:
            return True
        return False

    def filter_actions(self, actions: np.ndarray, kps: np.ndarray, kds: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, bool]:
        """Clamp and validate actions/gains. Returns (actions, kps, kds, force_damping)."""
        self._step += 1
        if not self.cfg.enable:
            return actions, kps, kds, False

        if not np.isfinite(actions).all():
            self._fault("actions NaN/Inf")
            return self._prev_action.copy(), self._prev_kp.copy(), self._prev_kd.copy(), True
        if not np.isfinite(kps).all() or not np.isfinite(kds).all():
            self._fault("gains NaN/Inf")
            return self._prev_action.copy(), self._prev_kp.copy(), self._prev_kd.copy(), True

        act = actions.astype(np.float32, copy=True)
        kp = kps.astype(np.float32, copy=True)
        kd = kds.astype(np.float32, copy=True)

        if self.cfg.max_action_abs is not None:
            act = np.clip(act, -self.cfg.max_action_abs, self.cfg.max_action_abs)
        if self.cfg.max_action_delta is not None:
            delta = act - self._prev_action
            delta = np.clip(delta, -self.cfg.max_action_delta, self.cfg.max_action_delta)
            act = self._prev_action + delta

        if self.cfg.max_kp is not None:
            kp = np.clip(kp, 0.0, self.cfg.max_kp)
        if self.cfg.max_kd is not None:
            kd = np.clip(kd, 0.0, self.cfg.max_kd)
        if self.cfg.max_kp_delta is not None:
            dkp = kp - self._prev_kp
            dkp = np.clip(dkp, -self.cfg.max_kp_delta, self.cfg.max_kp_delta)
            kp = self._prev_kp + dkp
        if self.cfg.max_kd_delta is not None:
            dkd = kd - self._prev_kd
            dkd = np.clip(dkd, -self.cfg.max_kd_delta, self.cfg.max_kd_delta)
            kd = self._prev_kd + dkd

        # update prev
        self._prev_action = act.copy()
        self._prev_kp = kp.copy()
        self._prev_kd = kd.copy()

        force_damping = self._should_force_damping()
        return act, kp, kd, force_damping
