#pragma once

#include "magicbot_loco_core.h"
#include "mode_manager.h"

#include <array>

namespace magicbot_loco {

struct ExternalPolicyInput {
    RobotSnapshot snapshot{};
    std::array<float, 3> velocity_command{0.0f, 0.0f, 0.0f};
    float dt_s{kDefaultPolicyDt};
};

struct ExternalPolicyOutput {
    JointArray target_motor{};
    JointArray kp_motor{};
    JointArray kd_motor{};
    JointArray tau_limit_motor{};
    bool override_gains{false};
    bool complete{false};
    ControlMode next_mode{ControlMode::Loco};
};

class ExternalPolicyAdapter {
public:
    virtual ~ExternalPolicyAdapter() = default;

    virtual ControlMode mode() const = 0;
    virtual const char* name() const = 0;
    virtual void reset(const RobotSnapshot& snapshot) = 0;
    virtual ExternalPolicyOutput step(const ExternalPolicyInput& input) = 0;
};

}  // namespace magicbot_loco
