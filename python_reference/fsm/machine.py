from shared.path_config import PROJECT_ROOT

from policies.passive.PassiveMode import PassiveMode
from policies.fixedpose.FixedPose import FixedPose
from policies.loco_mode.LocoMode import LocoMode
from policies.skill_cooldown.SkillCooldown import SkillCooldown
from policies.beyond_mimic.BeyondMimic import BeyondMimic
from policies.imu_calib.ImuCalib import ImuCalib
from fsm.state import *
import time
from shared.ctrlcomp import *
from enum import Enum, unique

@unique
class FSMMode(Enum):
    CHANGE = 1
    NORMAL = 2

class FSM:
    def __init__(self, state_cmd:StateAndCmd, policy_output:PolicyOutput):
        self.state_cmd = state_cmd
        self.policy_output = policy_output
        self.cur_policy : FSMState
        self.next_policy : FSMState
        
        self.FSMmode = FSMMode.NORMAL
        self.paused = False
        
        self.passive_mode = PassiveMode(state_cmd, policy_output)
        self.fixed_pose_1 = FixedPose(state_cmd, policy_output)
        self.loco_policy = LocoMode(state_cmd, policy_output)
        self.skill_cooldown_policy = SkillCooldown(state_cmd, policy_output)
        self.beyond_mimic_policy = BeyondMimic(state_cmd, policy_output)
        self.dance_policy = self.beyond_mimic_policy
        self.track_mimic_policy = self.beyond_mimic_policy
        self.imu_calib_policy = ImuCalib(state_cmd, policy_output, self.loco_policy)
        
        print("initialized Z1 24DoF policies.")
        
        self.cur_policy = self.passive_mode
        print("current policy is ", self.cur_policy.name_str)
        
        
        

    def _print_mode_hints(self, policy_name: FSMStateName):
        pause_hint = " UP=PAUSE"
        hints = {
            FSMStateName.PASSIVE: "[Hints] PASSIVE/DAMPING, START=POS_RESET, R1+A=LOCO",
            FSMStateName.FIXEDPOSE: "[Hints] R1+A=LOCO, L3=PASSIVE",
            FSMStateName.LOCOMODE: "[Hints] R1+X/L1+Y=BEYOND_MIMIC, L3=PASSIVE",
            FSMStateName.SKILL_BEYOND_MIMIC: "[Hints] R1+A=LOCO, L3=PASSIVE",
            FSMStateName.SKILL_TRACK_MIMIC: "[Hints] R1+A=LOCO, L3=PASSIVE",
            FSMStateName.IMU_CALIB: "[Hints] 自动回到LOCO 或 L3=PASSIVE",
            FSMStateName.SKILL_COOLDOWN: "[Hints] 自动回到LOCO 或 L3=PASSIVE",
        }
        msg = hints.get(policy_name)
        if msg:
            if policy_name in (FSMStateName.SKILL_BEYOND_MIMIC, FSMStateName.SKILL_TRACK_MIMIC):
                print(msg + pause_hint)
            else:
                print(msg)

    def _is_mimic_policy(self) -> bool:
        return self.cur_policy.name in (FSMStateName.SKILL_BEYOND_MIMIC, FSMStateName.SKILL_TRACK_MIMIC)

    def _set_pause(self, enable: bool):
        self.paused = bool(enable)
        self.state_cmd.pause = self.paused
        if self.paused:
            print("[Pause] ON: freeze reference frame.")
        else:
            print("[Pause] OFF: resume policy updates.")

    def run(self):
        start_time = time.time()
        # Handle pause toggle command.
        if self.state_cmd.skill_cmd == FSMCommand.PAUSE:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            if self._is_mimic_policy():
                self._set_pause(not self.paused)
            else:
                # Ignore pause outside mimic-like policies.
                self._set_pause(False)

        if self.paused and not self._is_mimic_policy():
            self._set_pause(False)
        if self.paused:
            # If user sends a command while paused, exit pause to allow mode switch.
            if self.state_cmd.skill_cmd != FSMCommand.INVALID:
                self._set_pause(False)
            else:
                # While paused, ignore commands and block mode switching,
                # but keep running inference in the current policy.
                self.state_cmd.skill_cmd = FSMCommand.INVALID

        if(self.FSMmode == FSMMode.NORMAL): 
            self.cur_policy.run()
            if not self.paused:
                nextPolicyName = self.cur_policy.checkChange()
                
                if(nextPolicyName != self.cur_policy.name):
                    # change policy
                    self.FSMmode = FSMMode.CHANGE
                    self.cur_policy.exit()
                    self.get_next_policy(nextPolicyName)
                    print("Switched to ", self.cur_policy.name_str)
                    self._print_mode_hints(self.cur_policy.name)

        elif(self.FSMmode == FSMMode.CHANGE):
            self.cur_policy.enter()
            self.FSMmode = FSMMode.NORMAL
            self.cur_policy.run()
            
        # self.absoluteWait(self.cur_policy.control_horzion,self.start_time)
        end_time = time.time()
        # print("time cusume: ", end_time - start_time)

    def absoluteWait(self, control_dt, start_time):
        end_time = time.time()
        delta_time = end_time - start_time
        if(delta_time < control_dt):
            time.sleep(control_dt - delta_time)
        else:
            print("inference time beyond control horzion!!!")
            
            
    def get_next_policy(self, policy_name:FSMStateName):
        if(policy_name == FSMStateName.PASSIVE):
            self.cur_policy = self.passive_mode
        elif((policy_name == FSMStateName.FIXEDPOSE)):
            self.cur_policy = self.fixed_pose_1
        elif((policy_name == FSMStateName.LOCOMODE)):
            self.cur_policy = self.loco_policy
        elif((policy_name == FSMStateName.SKILL_Dance)):
            self.cur_policy = self.beyond_mimic_policy
        elif((policy_name == FSMStateName.SKILL_COOLDOWN)):
            self.cur_policy = self.skill_cooldown_policy
        elif(
            policy_name
            in (
                FSMStateName.SKILL_CAST,
                FSMStateName.SKILL_KungFu,
                FSMStateName.SKILL_KICK,
                FSMStateName.SKILL_KungFu2,
            )
        ):
            print(f"[FSM][Z1] Legacy skill {policy_name.name} is disabled; routing to BeyondMimic.")
            self.cur_policy = self.beyond_mimic_policy
        elif((policy_name == FSMStateName.SKILL_BEYOND_MIMIC)):
            self.cur_policy = self.beyond_mimic_policy
        elif((policy_name == FSMStateName.SKILL_TRACK_MIMIC)):
            self.cur_policy = self.beyond_mimic_policy
        elif((policy_name == FSMStateName.IMU_CALIB)):
            self.cur_policy = self.imu_calib_policy
        elif((policy_name == FSMStateName.JOINT_ZERO_CHECK)):
            self.cur_policy = self.fixed_pose_1
        else:
            pass
            
        
        
