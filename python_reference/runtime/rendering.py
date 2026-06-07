"""Rendering adapters for simulation; real deployment should use headless mode."""

from __future__ import annotations

from contextlib import nullcontext


def mujoco_viewer_context(model, data, headless: bool):
    """Return a MuJoCo viewer context or a no-op context for headless runs."""
    if headless:
        return nullcontext(None)
    import mujoco.viewer

    return mujoco.viewer.launch_passive(model, data)

