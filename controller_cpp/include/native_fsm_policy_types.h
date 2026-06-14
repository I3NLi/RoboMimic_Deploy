#pragma once

#include "magicbot_loco_core.h"
#include "native_fsm_states.h"

#include <array>
#include <string>

static constexpr int Z1_NUM_MOTOR = magicbot_loco::kNumJoints;

struct StateAndCmd {
    int num_joints;

    std::array<float, Z1_NUM_MOTOR> q{};
    std::array<float, Z1_NUM_MOTOR> dq{};
    std::array<float, Z1_NUM_MOTOR> ddq{};
    std::array<float, Z1_NUM_MOTOR> tau_est{};

    std::array<float, 3> gravity_ori{0.0f, 0.0f, -1.0f};
    std::array<float, 3> ang_vel{};
    std::array<float, 4> base_quat{1.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 3> vel_cmd{};
    int policy_step_override{-1};

    FSMCommand skill_cmd{FSMCommand::INVALID};
    bool pause{false};

    explicit StateAndCmd(int n) : num_joints(n) {}
};

struct PolicyOutput {
    int num_joints;
    std::array<float, Z1_NUM_MOTOR> actions{};
    std::array<float, Z1_NUM_MOTOR> kps{};
    std::array<float, Z1_NUM_MOTOR> kds{};

    explicit PolicyOutput(int n) : num_joints(n) {}
};

class FSMState {
public:
    FSMStateName name;
    std::string name_str;

    FSMState(FSMStateName n, const std::string& ns, StateAndCmd& sc, PolicyOutput& po)
        : name(n), name_str(ns), sc_(sc), po_(po)
    {
    }

    virtual ~FSMState() = default;
    virtual void enter() {}
    virtual void run() = 0;
    virtual void exit() {}
    virtual FSMStateName check_change() { return name; }

protected:
    StateAndCmd& sc_;
    PolicyOutput& po_;
};
