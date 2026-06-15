#!/usr/bin/env python3
"""Create a temporary TrackMimic YAML with a minimal external motion file."""

from __future__ import annotations

import argparse
import struct
import sys
import zipfile
from array import array
from pathlib import Path


def without_root_motion_file(text: str) -> str:
    kept: list[str] = []
    for line in text.splitlines():
        if line.lstrip() == line and line.split(":", 1)[0].strip() == "motion_file":
            continue
        kept.append(line)
    return "\n".join(kept).rstrip()


def yaml_scalar(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def parse_top_level_scalar(text: str, key: str) -> str | None:
    prefix = f"{key}:"
    for line in text.splitlines():
        if line.lstrip() != line or not line.startswith(prefix):
            continue
        value = line[len(prefix):].strip()
        if (value.startswith('"') and value.endswith('"')) or (
            value.startswith("'") and value.endswith("'")
        ):
            return value[1:-1]
        return value or None
    return None


def resolve_onnx_path(base_yaml: Path, base_text: str) -> Path | None:
    raw = parse_top_level_scalar(base_text, "onnx_path")
    if not raw:
        return None
    onnx_path = Path(raw).expanduser()
    if onnx_path.is_absolute():
        return onnx_path.resolve()

    yaml_dir = base_yaml.parent
    candidate_model = yaml_dir.parent / "model" / onnx_path
    candidate_same_dir = yaml_dir / onnx_path
    if candidate_model.exists():
        return candidate_model.resolve()
    if candidate_same_dir.exists():
        return candidate_same_dir.resolve()
    return candidate_model.resolve()


def with_absolute_onnx_path(text: str, onnx_path: Path | None) -> str:
    if onnx_path is None:
        return text
    lines: list[str] = []
    replaced = False
    for line in text.splitlines():
        if line.lstrip() == line and line.startswith("onnx_path:"):
            lines.append(f"onnx_path: {yaml_scalar(onnx_path.as_posix())}")
            replaced = True
        else:
            lines.append(line)
    if not replaced:
        lines.insert(0, f"onnx_path: {yaml_scalar(onnx_path.as_posix())}")
    return "\n".join(lines).rstrip()


def npy_f32(shape: tuple[int, ...], values: list[float]) -> bytes:
    count = 1
    for dim in shape:
        count *= dim
    if len(values) != count:
        raise ValueError(f"shape {shape} needs {count} values, got {len(values)}")

    shape_repr = f"({shape[0]},)" if len(shape) == 1 else "(" + ", ".join(str(v) for v in shape) + ")"
    header = (
        "{'descr': '<f4', 'fortran_order': False, 'shape': "
        + shape_repr
        + ", }"
    )
    header_bytes = header.encode("latin1")
    padding = 16 - ((10 + len(header_bytes) + 1) % 16)
    header_bytes += b" " * padding + b"\n"
    payload = array("f", values)
    if sys.byteorder != "little":
        payload.byteswap()
    return b"\x93NUMPY" + bytes([1, 0]) + struct.pack("<H", len(header_bytes)) + header_bytes + payload.tobytes()


def write_motion_npz(path: Path) -> None:
    steps = 4096
    num_actions = 24
    body_count = 14

    joint_pos = [0.0] * (steps * num_actions)
    joint_vel = [0.0] * (steps * num_actions)
    body_quat_w = [0.0] * (steps * body_count * 4)
    for step in range(steps):
        for body in range(body_count):
            body_quat_w[(step * body_count + body) * 4] = 1.0

    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_STORED) as zf:
        zf.writestr("joint_pos.npy", npy_f32((steps, num_actions), joint_pos))
        zf.writestr("joint_vel.npy", npy_f32((steps, num_actions), joint_vel))
        zf.writestr("body_quat_w.npy", npy_f32((steps, body_count, 4), body_quat_w))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-yaml", required=True)
    parser.add_argument("--output-yaml", required=True)
    parser.add_argument("--motion-file")
    args = parser.parse_args()

    base_yaml = Path(args.base_yaml).expanduser().resolve()
    output_yaml = Path(args.output_yaml).expanduser().resolve()
    motion_file = (
        Path(args.motion_file).expanduser().resolve()
        if args.motion_file
        else output_yaml.with_name("track_mimic_smoke_motion.npz")
    )

    if not base_yaml.is_file():
        raise SystemExit(f"base YAML not found: {base_yaml}")

    motion_file.parent.mkdir(parents=True, exist_ok=True)
    output_yaml.parent.mkdir(parents=True, exist_ok=True)

    write_motion_npz(motion_file)

    base_text_raw = base_yaml.read_text()
    base_text = with_absolute_onnx_path(
        without_root_motion_file(base_text_raw),
        resolve_onnx_path(base_yaml, base_text_raw),
    )
    output_yaml.write_text(
        base_text
        + "\n\n# Generated for smoke tests; TrackMimic requires an external trajectory.\n"
        + f'motion_file: "{motion_file.as_posix()}"\n'
    )
    print(output_yaml)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
