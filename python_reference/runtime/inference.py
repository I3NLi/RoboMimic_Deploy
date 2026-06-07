"""Inference/FSM helpers that do not know about rendering or transport."""

from __future__ import annotations

from fsm.machine import FSM, FSMMode
from fsm.state import FSMStateName


def force_fsm_state(fsm: FSM, state_name: FSMStateName):
    """Force Python FSM into a state, mirroring C++ shadow mode behavior."""
    if hasattr(fsm, "force_state"):
        fsm.force_state(state_name)
        return

    try:
        fsm.cur_policy.exit()
    except Exception:
        pass
    fsm.get_next_policy(state_name)
    fsm.FSMmode = FSMMode.CHANGE
    print(f"[FSM] force_state -> {state_name.name}")
