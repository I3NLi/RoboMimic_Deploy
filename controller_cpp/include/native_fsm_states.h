#pragma once

#include "mode_manager.h"

enum class FSMCommand {
    INVALID = -1,
    POS_RESET = 1,
    LOCO = 2,
    PASSIVE = 4,
    SKILL_1 = 5,
    SKILL_2 = 6,
    SKILL_3 = 7,
    SKILL_4 = 8,
    SKILL_5 = 9,
    SKILL_6 = 10,
    SKILL_7 = 11,
    PAUSE = 12,
};

enum class FSMStateName {
    INVALID = -1,
    PASSIVE = 1,
    FIXEDPOSE = 2,
    SKILL_COOLDOWN = 3,
    LOCOMODE = 4,
    SKILL_CAST = 5,
    SKILL_KUNGFU = 6,
    SKILL_DANCE = 7,
    SKILL_KICK = 8,
    SKILL_KUNGFU2 = 9,
    SKILL_BEYOND_MIMIC = 10,
    JOINT_ZERO_CHECK = 11,
    IMU_CALIB = 12,
    SKILL_TRACK_MIMIC = 13,
};

inline magicbot_loco::ControlMode control_mode_for_fsm_state(FSMStateName state)
{
    switch (state) {
    case FSMStateName::PASSIVE:
        return magicbot_loco::ControlMode::Passive;
    case FSMStateName::FIXEDPOSE:
        return magicbot_loco::ControlMode::Stand;
    case FSMStateName::LOCOMODE:
    case FSMStateName::SKILL_COOLDOWN:
        return magicbot_loco::ControlMode::Loco;
    case FSMStateName::SKILL_BEYOND_MIMIC:
    case FSMStateName::SKILL_DANCE:
        return magicbot_loco::ControlMode::Dance;
    case FSMStateName::SKILL_TRACK_MIMIC:
        return magicbot_loco::ControlMode::Skill;
    default:
        return magicbot_loco::ControlMode::Skill;
    }
}
