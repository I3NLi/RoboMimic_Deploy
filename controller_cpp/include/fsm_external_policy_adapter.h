#pragma once

#include "policy_adapter.h"

#include <algorithm>
#include <array>
#include <functional>
#include <string>
#include <utility>

namespace magicbot_loco {

template <typename FsmStateT, typename StateAndCmdT, typename PolicyOutputT, typename StateNameT>
class FsmExternalPolicyAdapter final : public ExternalPolicyAdapter {
public:
    using NextModeMapper = std::function<ControlMode(StateNameT)>;

    FsmExternalPolicyAdapter(
        ControlMode mode,
        std::string name,
        StateNameT self_state,
        StateAndCmdT& state_cmd,
        PolicyOutputT& policy_output,
        FsmStateT& policy,
        NextModeMapper next_mode_mapper,
        bool override_gains = true)
        : mode_(mode),
          name_(std::move(name)),
          self_state_(self_state),
          state_cmd_(state_cmd),
          policy_output_(policy_output),
          policy_(policy),
          next_mode_mapper_(std::move(next_mode_mapper)),
          override_gains_(override_gains)
    {
    }

    ControlMode mode() const override { return mode_; }

    const char* name() const override { return name_.c_str(); }

    void reset(const RobotSnapshot& snapshot) override
    {
        if (entered_) policy_.exit();
        copy_snapshot(snapshot, {});
        policy_.enter();
        entered_ = true;
    }

    ExternalPolicyOutput step(const ExternalPolicyInput& input) override
    {
        if (!entered_) reset(input.snapshot);
        copy_snapshot(input.snapshot, input.velocity_command);
        policy_.run();

        ExternalPolicyOutput out;
        for (int i = 0; i < kNumJoints; ++i) {
            out.target_motor[i] = policy_output_.actions[static_cast<size_t>(i)];
            out.kp_motor[i] = policy_output_.kps[static_cast<size_t>(i)];
            out.kd_motor[i] = policy_output_.kds[static_cast<size_t>(i)];
        }
        out.override_gains = override_gains_;

        const StateNameT next_state = policy_.check_change();
        if (next_state != self_state_) {
            out.complete = true;
            out.next_mode = next_mode_mapper_ ? next_mode_mapper_(next_state) : ControlMode::Loco;
        }
        return out;
    }

private:
    void copy_snapshot(const RobotSnapshot& snapshot, const std::array<float, 3>& command)
    {
        const int n = std::min(state_cmd_.num_joints, kNumJoints);
        for (int i = 0; i < n; ++i) {
            state_cmd_.q[static_cast<size_t>(i)] = snapshot.q[static_cast<size_t>(i)];
            state_cmd_.dq[static_cast<size_t>(i)] = snapshot.dq[static_cast<size_t>(i)];
        }
        state_cmd_.base_quat = snapshot.quat;
        state_cmd_.gravity_ori = gravity_orientation(snapshot.quat);
        state_cmd_.ang_vel = snapshot.ang_vel;
        state_cmd_.vel_cmd = command;
    }

    ControlMode mode_{ControlMode::Skill};
    std::string name_;
    StateNameT self_state_;
    StateAndCmdT& state_cmd_;
    PolicyOutputT& policy_output_;
    FsmStateT& policy_;
    NextModeMapper next_mode_mapper_;
    bool override_gains_{true};
    bool entered_{false};
};

}  // namespace magicbot_loco
