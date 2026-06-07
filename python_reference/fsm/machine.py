"""Runtime FSM for the Python reference controller.

The policy classes still own their low-level control math. This module owns
state registration, command-level pause behavior, and transition scheduling.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, unique
from typing import Dict, Iterable, Optional, Set

from fsm.state import FSMState
from policies.beyond_mimic.BeyondMimic import BeyondMimic
from policies.fixedpose.FixedPose import FixedPose
from policies.imu_calib.ImuCalib import ImuCalib
from policies.joint_zero_check.JointZeroCheck import JointZeroCheck
from policies.loco_mode.LocoMode import LocoMode
from policies.passive.PassiveMode import PassiveMode
from policies.skill_cooldown.SkillCooldown import SkillCooldown
from shared.ctrlcomp import PolicyOutput, StateAndCmd
from shared.utils import FSMCommand, FSMStateName


@unique
class FSMMode(Enum):
    CHANGE = 1
    NORMAL = 2


@dataclass(frozen=True)
class PolicyRecord:
    policy: FSMState
    mimic: bool = False
    hint: Optional[str] = None


class FSM:
    """Small, explicit FSM shell around policy implementations.

    Existing policy classes expose ``enter/run/exit/checkChange``. The FSM keeps
    that interface intact while centralizing the policy registry, alias routing,
    force-state handling, and pause semantics.
    """

    def __init__(self, state_cmd: StateAndCmd, policy_output: PolicyOutput):
        self.state_cmd = state_cmd
        self.policy_output = policy_output

        self.FSMmode = FSMMode.NORMAL
        self.paused = False
        self.cur_policy: FSMState
        self.next_policy: Optional[FSMState] = None
        self.current_state_name = FSMStateName.PASSIVE

        self._records: Dict[FSMStateName, PolicyRecord] = {}
        self._aliases: Dict[FSMStateName, FSMStateName] = {}
        self._mimic_states: Set[FSMStateName] = set()
        self._warned_missing: Set[FSMStateName] = set()
        self._warned_legacy_route: Set[FSMStateName] = set()

        self.passive_mode = PassiveMode(state_cmd, policy_output)
        self.fixed_pose_1 = FixedPose(state_cmd, policy_output)
        self.loco_policy = LocoMode(state_cmd, policy_output)
        self.skill_cooldown_policy = SkillCooldown(state_cmd, policy_output)
        self.beyond_mimic_policy = BeyondMimic(state_cmd, policy_output)
        self.dance_policy = self.beyond_mimic_policy
        self.track_mimic_policy = self.beyond_mimic_policy
        self.joint_zero_check_policy = JointZeroCheck(state_cmd, policy_output)
        self.imu_calib_policy = ImuCalib(state_cmd, policy_output, self.loco_policy)

        self._register_builtin_policies()

        self.cur_policy = self.passive_mode
        print("[FSM] initialized Z1 24DoF policies.")
        print("[FSM] current policy is", self.cur_policy.name_str)

    def register_policy(
        self,
        state_name: FSMStateName,
        policy: FSMState,
        *,
        aliases: Iterable[FSMStateName] = (),
        mimic: bool = False,
        hint: Optional[str] = None,
    ) -> None:
        """Register or replace a policy implementation for one FSM state."""
        self._records[state_name] = PolicyRecord(policy=policy, mimic=mimic, hint=hint)
        self._aliases[state_name] = state_name
        if mimic:
            self._mimic_states.add(state_name)
        for alias in aliases:
            self._aliases[alias] = state_name
            if mimic:
                self._mimic_states.add(alias)

    def force_state(self, state_name: FSMStateName) -> None:
        """Jump to a state and schedule ``enter`` on the next ``run`` call."""
        self._set_pause(False)
        try:
            self.cur_policy.exit()
        except Exception:
            pass
        self._select_policy(state_name)
        self.FSMmode = FSMMode.CHANGE
        self.state_cmd.skill_cmd = FSMCommand.INVALID
        print(f"[FSM] force_state -> {self.current_state_name.name}")

    def run(self) -> None:
        self._handle_pause_command()

        if self.paused and not self._is_mimic_policy():
            self._set_pause(False)
        if self.paused:
            if self.state_cmd.skill_cmd != FSMCommand.INVALID:
                self._set_pause(False)
            else:
                self.state_cmd.skill_cmd = FSMCommand.INVALID

        if self.FSMmode == FSMMode.CHANGE:
            self.cur_policy.enter()
            self.FSMmode = FSMMode.NORMAL
            self.cur_policy.run()
            return

        self.cur_policy.run()
        if self.paused:
            return

        requested_state = self.cur_policy.checkChange()
        self._transition_if_needed(requested_state)

    def get_next_policy(self, policy_name: FSMStateName):
        """Compatibility wrapper used by older scripts.

        It only selects the policy object. Callers that need an enter pass should
        set ``FSMmode = FSMMode.CHANGE`` or use ``force_state``.
        """
        self._select_policy(policy_name)
        return self.cur_policy

    def absoluteWait(self, control_dt, start_time):
        import time

        delta_time = time.time() - start_time
        if delta_time < control_dt:
            time.sleep(control_dt - delta_time)
        else:
            print("inference time beyond control horzion!!!")

    def _register_builtin_policies(self) -> None:
        self.register_policy(
            FSMStateName.PASSIVE,
            self.passive_mode,
            hint="[Hints] PASSIVE/DAMPING, START=POS_RESET, R1+A=LOCO",
        )
        self.register_policy(
            FSMStateName.FIXEDPOSE,
            self.fixed_pose_1,
            hint="[Hints] R1+A=LOCO, L3=PASSIVE",
        )
        self.register_policy(
            FSMStateName.LOCOMODE,
            self.loco_policy,
            hint="[Hints] R1+X/L1+Y=BEYOND_MIMIC, L3=PASSIVE",
        )
        self.register_policy(
            FSMStateName.SKILL_COOLDOWN,
            self.skill_cooldown_policy,
            hint="[Hints] auto return to LOCO or L3=PASSIVE",
        )
        self.register_policy(
            FSMStateName.SKILL_BEYOND_MIMIC,
            self.beyond_mimic_policy,
            aliases=(
                FSMStateName.SKILL_CAST,
                FSMStateName.SKILL_KungFu,
                FSMStateName.SKILL_Dance,
                FSMStateName.SKILL_KICK,
                FSMStateName.SKILL_KungFu2,
                FSMStateName.SKILL_TRACK_MIMIC,
            ),
            mimic=True,
            hint="[Hints] R1+A=LOCO, L3=PASSIVE, UP=PAUSE",
        )
        self.register_policy(
            FSMStateName.JOINT_ZERO_CHECK,
            self.joint_zero_check_policy,
            hint="[Hints] joint-zero check, R1+A=LOCO, L3=PASSIVE",
        )
        self.register_policy(
            FSMStateName.IMU_CALIB,
            self.imu_calib_policy,
            hint="[Hints] auto return to LOCO or L3=PASSIVE",
        )

    def _handle_pause_command(self) -> None:
        if self.state_cmd.skill_cmd != FSMCommand.PAUSE:
            return
        self.state_cmd.skill_cmd = FSMCommand.INVALID
        if self._is_mimic_policy():
            self._set_pause(not self.paused)
        else:
            self._set_pause(False)

    def _transition_if_needed(self, requested_state: FSMStateName) -> None:
        if requested_state is None:
            requested_state = self.current_state_name
        target_state = self._normalize_state(requested_state)
        target_policy = self._policy_for_state(target_state)
        if target_policy is self.cur_policy and target_state == self.current_state_name:
            return

        self._set_pause(False)
        self.FSMmode = FSMMode.CHANGE
        self.cur_policy.exit()
        self.current_state_name = target_state
        self.cur_policy = target_policy
        print("[FSM] switched to", self.cur_policy.name_str)
        self._print_mode_hints(target_state)

    def _select_policy(self, requested_state: FSMStateName) -> None:
        target_state = self._normalize_state(requested_state)
        self.current_state_name = target_state
        self.cur_policy = self._policy_for_state(target_state)

    def _normalize_state(self, requested_state: FSMStateName) -> FSMStateName:
        target_state = self._aliases.get(requested_state)
        if target_state is not None:
            if target_state == FSMStateName.SKILL_BEYOND_MIMIC and requested_state != target_state:
                if requested_state not in self._warned_legacy_route:
                    self._warned_legacy_route.add(requested_state)
                    print(
                        f"[FSM][Z1] {requested_state.name} is routed to "
                        "SKILL_BEYOND_MIMIC in the Python reference."
                    )
            return target_state

        if requested_state not in self._warned_missing:
            self._warned_missing.add(requested_state)
            print(
                f"[FSM][WARN] unregistered state {requested_state}; "
                f"keeping {self.current_state_name.name}"
            )
        return self.current_state_name

    def _policy_for_state(self, state_name: FSMStateName) -> FSMState:
        record = self._records.get(state_name)
        if record is None:
            return self.cur_policy
        return record.policy

    def _print_mode_hints(self, state_name: FSMStateName) -> None:
        record = self._records.get(state_name)
        if record and record.hint:
            print(record.hint)

    def _is_mimic_policy(self) -> bool:
        return self.current_state_name in self._mimic_states

    def _set_pause(self, enable: bool) -> None:
        enable = bool(enable)
        if self.paused == enable and self.state_cmd.pause == enable:
            return
        self.paused = enable
        self.state_cmd.pause = enable
        if self.paused:
            print("[Pause] ON: freeze reference frame.")
        else:
            print("[Pause] OFF: resume policy updates.")
