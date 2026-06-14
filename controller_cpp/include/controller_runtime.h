#pragma once

#include "controller_core.h"
#include "robot_adapter.h"

namespace magicbot_loco {

struct RuntimeTickInput {
    Command command{};
    ModeRequest mode_request{ModeRequest::none()};
    float control_dt_s{static_cast<float>(kControlDt)};
    bool publish_target{true};
};

struct RuntimeTickOutput {
    RobotSnapshot snapshot{};
    ControllerCoreOutput core{};
    AdapterTelemetry adapter{};
};

class ControllerRuntime {
public:
    ControllerRuntime(ControllerCore& core, RobotAdapter& adapter)
        : core_(core),
          adapter_(adapter)
    {
    }

    RuntimeTickOutput tick(const RuntimeTickInput& input)
    {
        RuntimeTickOutput out;
        out.snapshot = adapter_.read_snapshot();
        out.core = core_.step(out.snapshot, input.command, input.mode_request, input.control_dt_s);
        if (input.publish_target) {
            adapter_.write_target(out.core.target);
        }
        out.adapter = adapter_.telemetry();
        return out;
    }

    void write_target(const JointTarget& target)
    {
        adapter_.write_target(target);
    }

    void write_damping(float damping_kd)
    {
        adapter_.write_damping(damping_kd);
    }

private:
    ControllerCore& core_;
    RobotAdapter& adapter_;
};

}  // namespace magicbot_loco
