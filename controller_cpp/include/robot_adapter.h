#pragma once

#include "controller_core.h"

#include <string>

namespace magicbot_loco {

struct AdapterTelemetry {
    std::string backend;
    double state_age_ms{-1.0};
    bool command_published{false};
};

class RobotAdapter {
public:
    virtual ~RobotAdapter() = default;

    virtual const char* name() const = 0;
    virtual RobotSnapshot read_snapshot() = 0;
    virtual void write_target(const JointTarget& target) = 0;
    virtual void write_damping(float damping_kd) = 0;

    virtual AdapterTelemetry telemetry() const
    {
        AdapterTelemetry out;
        out.backend = name();
        return out;
    }
};

}  // namespace magicbot_loco
