"""Control math that is independent from rendering and communication."""

from __future__ import annotations

import numpy as np


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculate PD torques from position/velocity targets."""
    return (target_q - q) * kp + (target_dq - dq) * kd


def sanitize_ctrl(ctrl, model=None, fallback=None, ctrl_limit=None):
    """Ensure finite actuator controls and clamp to actuator limits."""
    out = np.asarray(ctrl, dtype=np.float32).reshape(-1)
    if fallback is None:
        fallback = np.zeros_like(out)
    fallback = np.asarray(fallback, dtype=np.float32).reshape(-1)
    if fallback.size != out.size:
        fallback = np.zeros_like(out)

    finite_mask = np.isfinite(out)
    if not np.all(finite_mask):
        out = np.where(finite_mask, out, fallback)
    out = np.nan_to_num(out, nan=0.0, posinf=0.0, neginf=0.0)

    if ctrl_limit is not None:
        limit = np.asarray(ctrl_limit, dtype=np.float32).reshape(-1)
        if limit.size == out.size and np.any(limit > 0.0):
            limit = np.maximum(limit, 0.0)
            return np.clip(out, -limit, limit).astype(np.float32)

    ctrl_range = getattr(model, "actuator_ctrlrange", None) if model is not None else None
    if ctrl_range is not None and ctrl_range.shape[0] == out.size:
        lo = ctrl_range[:, 0]
        hi = ctrl_range[:, 1]
        if np.any(hi > lo):
            return np.clip(out, lo, hi).astype(np.float32)

    return np.clip(out, -300.0, 300.0).astype(np.float32)

