from common.path_config import PROJECT_ROOT

from policy.passive.PassiveMode import PassiveMode
from policy.fixedpose.FixedPose import FixedPose
from policy.loco_mode.LocoMode import LocoMode
from policy.kungfu.KungFu import KungFu
from policy.dance.Dance import Dance
from policy.skill_cooldown.SkillCooldown import SkillCooldown
from policy.skill_cast.SkillCast import SkillCast
from policy.kick.Kick import Kick
from policy.kungfu2.KungFu2 import KungFu2
from policy.beyond_mimic.BeyondMimic import BeyondMimic
from policy.track_mimic.BeyondMimic import TrackMimic
from policy.imu_calib.ImuCalib import ImuCalib
from policy.joint_zero_check.JointZeroCheck import JointZeroCheck
from FSM.FSMState import *
import time
from common.ctrlcomp import *
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
        self.kungfu_policy = KungFu(state_cmd, policy_output)
        self.dance_policy = Dance(state_cmd, policy_output)
        self.skill_cooldown_policy = SkillCooldown(state_cmd, policy_output)
        self.skill_cast_policy = SkillCast(state_cmd, policy_output)
        self.kick_policy = Kick(state_cmd, policy_output)
        self.kungfu2_policy = KungFu2(state_cmd, policy_output)
        self.beyond_mimic_policy = BeyondMimic(state_cmd, policy_output)
        self.track_mimic_policy = TrackMimic(state_cmd, policy_output)
        self.imu_calib_policy = ImuCalib(state_cmd, policy_output, self.loco_policy)
        self.joint_zero_check_policy = JointZeroCheck(state_cmd, policy_output)
        
        print("initalized all policies!!!")
        
        self.cur_policy = self.passive_mode
        print("current policy is ", self.cur_policy.name_str)
        
        
        

    def _print_mode_hints(self, policy_name: FSMStateName):
        pause_hint = " UP=PAUSE"
        hints = {
            FSMStateName.PASSIVE: "[Hints] START=POS_RESET, R1+A=LOCO",
            FSMStateName.FIXEDPOSE: "[Hints] R1+A=LOCO, L3=PASSIVE",
            FSMStateName.LOCOMODE: "[Hints] R1+X=SKILL_1, R1+Y=SKILL_2, R1+B=SKILL_3, L1+Y=SKILL_4, L1+B=SKILL_5, L1+X=SKILL_6, L1+A=SKILL_7, L3=PASSIVE",
            FSMStateName.SKILL_Dance: "[Hints] R1+A=LOCO, L3=PASSIVE",
            FSMStateName.SKILL_KungFu: "[Hints] R1+A=LOCO, L3=PASSIVE",
            FSMStateName.SKILL_KICK: "[Hints] R1+A=LOCO, L3=PASSIVE",
            FSMStateName.SKILL_KungFu2: "[Hints] R1+A=LOCO, L3=PASSIVE",
            FSMStateName.SKILL_BEYOND_MIMIC: "[Hints] R1+A=LOCO, L3=PASSIVE",
            FSMStateName.SKILL_TRACK_MIMIC: "[Hints] R1+A=LOCO, L3=PASSIVE",
            FSMStateName.JOINT_ZERO_CHECK: "[Hints] R1+A=LOCO, L3=PASSIVE",
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
        elif((policy_name == FSMStateName.SKILL_KungFu)):
            self.cur_policy = self.kungfu_policy
        elif((policy_name == FSMStateName.SKILL_Dance)):
            self.cur_policy = self.dance_policy
        elif((policy_name == FSMStateName.SKILL_COOLDOWN)):
            self.cur_policy = self.skill_cooldown_policy
        elif((policy_name == FSMStateName.SKILL_CAST)):
            self.cur_policy = self.skill_cast_policy
        elif((policy_name == FSMStateName.SKILL_KICK)):
            self.cur_policy = self.kick_policy
        elif((policy_name == FSMStateName.SKILL_KungFu2)):
            self.cur_policy = self.kungfu2_policy
        elif((policy_name == FSMStateName.SKILL_BEYOND_MIMIC)):
            self.cur_policy = self.beyond_mimic_policy
        elif((policy_name == FSMStateName.SKILL_TRACK_MIMIC)):
            self.cur_policy = self.track_mimic_policy
        elif((policy_name == FSMStateName.IMU_CALIB)):
            self.cur_policy = self.imu_calib_policy
        elif((policy_name == FSMStateName.JOINT_ZERO_CHECK)):
            self.cur_policy = self.joint_zero_check_policy
        else:
            pass
            
        
        
