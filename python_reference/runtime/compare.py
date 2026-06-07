"""Python/C++ policy comparison utilities."""

from __future__ import annotations

import numpy as np


class DiffLogger:
    """Print and optionally CSV-log per-step diffs between two command streams."""

    def __init__(self, csv_path: str | None = None, print_every: int = 20):
        self._f = None
        self.print_every = max(1, int(print_every))
        self.cmp_steps = 0
        self.max_q = 0.0
        self.max_mean_q = 0.0
        self.max_kp = 0.0
        self.max_kd = 0.0
        self.max_lag = 0
        self.sum_lag = 0
        self.last = {
            "step": -1,
            "cpp_recv": 0,
            "max_q": 0.0,
            "mean_q": 0.0,
            "max_kp": 0.0,
            "max_kd": 0.0,
            "lag_steps": 0,
        }
        if csv_path:
            self._f = open(csv_path, "w", buffering=1)
            self._f.write("step,cpp_recv,max_q,mean_q,max_kp,max_kd,lag_steps\n")

    def log(
        self,
        step: int,
        cpp_recv: int,
        py_q: np.ndarray,
        cpp_q: np.ndarray,
        py_kp: np.ndarray,
        cpp_kp: np.ndarray,
        py_kd: np.ndarray,
        cpp_kd: np.ndarray,
        lag_steps: int = 0,
    ):
        dq = np.abs(py_q - cpp_q)
        dkp = np.abs(py_kp - cpp_kp)
        dkd = np.abs(py_kd - cpp_kd)
        max_q = float(np.max(dq))
        mean_q = float(np.mean(dq))
        max_kp = float(np.max(dkp))
        max_kd = float(np.max(dkd))
        max_q_idx = int(np.argmax(dq))
        self.cmp_steps += 1
        self.max_q = max(self.max_q, max_q)
        self.max_mean_q = max(self.max_mean_q, mean_q)
        self.max_kp = max(self.max_kp, max_kp)
        self.max_kd = max(self.max_kd, max_kd)
        self.max_lag = max(self.max_lag, int(abs(lag_steps)))
        self.sum_lag += int(lag_steps)
        self.last = {
            "step": int(step),
            "cpp_recv": int(cpp_recv),
            "max_q": max_q,
            "mean_q": mean_q,
            "max_kp": max_kp,
            "max_kd": max_kd,
            "lag_steps": int(lag_steps),
        }
        if (self.cmp_steps % self.print_every) == 0:
            print(
                f"[step {step:6d} | cpp#{cpp_recv:6d}]  "
                f"q diff  max={max_q:.6f}  mean={mean_q:.6f}  |  "
                f"kp max={max_kp:.6f}  kd max={max_kd:.6f}  "
                f"lag={lag_steps:+d}  joint={max_q_idx}"
            )
        if self._f:
            self._f.write(
                f"{step},{cpp_recv},{max_q:.6f},{mean_q:.6f},"
                f"{max_kp:.6f},{max_kd:.6f},{lag_steps}\n"
            )

    def summary(self) -> dict:
        return {
            "cmp_steps": int(self.cmp_steps),
            "max_q": float(self.max_q),
            "max_mean_q": float(self.max_mean_q),
            "max_kp": float(self.max_kp),
            "max_kd": float(self.max_kd),
            "max_lag_steps": int(self.max_lag),
            "avg_lag_steps": float(self.sum_lag / self.cmp_steps) if self.cmp_steps > 0 else 0.0,
            "last": dict(self.last),
        }

    def close(self):
        if self._f:
            self._f.close()

