#!/usr/bin/env python3
"""Benchmark ONNXRuntime inference latency for one- or two-input policies."""

from __future__ import annotations

import argparse
import statistics
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort


def shape_from_input(inp, fallback_dim: int | None = None) -> tuple[int, ...]:
    shape = []
    for i, dim in enumerate(inp.shape):
        if isinstance(dim, int) and dim > 0:
            shape.append(dim)
        elif i == 0:
            shape.append(1)
        elif fallback_dim is not None:
            shape.append(fallback_dim)
        else:
            shape.append(1)
    return tuple(shape)


def make_inputs(session: ort.InferenceSession, obs_dim: int | None):
    feeds = {}
    for i, inp in enumerate(session.get_inputs()):
        fallback = obs_dim if i == 0 else 1
        shape = shape_from_input(inp, fallback)
        if i == 1 and np.prod(shape) == 1:
            feeds[inp.name] = np.array([[0.0]], dtype=np.float32)
        else:
            feeds[inp.name] = np.zeros(shape, dtype=np.float32)
    return feeds


def percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    idx = min(len(values) - 1, max(0, int(round((len(values) - 1) * q))))
    return sorted(values)[idx]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--iters", type=int, default=5000)
    parser.add_argument("--warmup", type=int, default=200)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--obs-dim", type=int, default=None)
    args = parser.parse_args()

    opts = ort.SessionOptions()
    opts.intra_op_num_threads = int(args.threads)
    opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    session = ort.InferenceSession(str(args.model), sess_options=opts, providers=["CPUExecutionProvider"])
    feeds = make_inputs(session, args.obs_dim)
    output_names = [out.name for out in session.get_outputs()]

    for i in range(max(0, args.warmup)):
        if len(feeds) >= 2:
            second_key = list(feeds.keys())[1]
            feeds[second_key][0, 0] = float(i)
        session.run(output_names, feeds)

    samples = []
    t_total0 = time.perf_counter()
    for i in range(max(1, args.iters)):
        if len(feeds) >= 2:
            second_key = list(feeds.keys())[1]
            feeds[second_key][0, 0] = float(i)
        t0 = time.perf_counter()
        session.run(output_names, feeds)
        samples.append((time.perf_counter() - t0) * 1000.0)
    total_ms = (time.perf_counter() - t_total0) * 1000.0

    print(f"runtime=python")
    print(f"model={args.model}")
    print(f"inputs={[(i.name, i.shape) for i in session.get_inputs()]}")
    print(f"outputs={[(o.name, o.shape) for o in session.get_outputs()]}")
    print(f"threads={args.threads} warmup={args.warmup} iters={args.iters}")
    print(f"mean_ms={statistics.fmean(samples):.6f}")
    print(f"median_ms={statistics.median(samples):.6f}")
    print(f"p95_ms={percentile(samples, 0.95):.6f}")
    print(f"p99_ms={percentile(samples, 0.99):.6f}")
    print(f"min_ms={min(samples):.6f}")
    print(f"max_ms={max(samples):.6f}")
    print(f"throughput_hz={args.iters / max(total_ms / 1000.0, 1e-9):.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

