#pragma once

#include "magicbot_loco_sdk_adapter.h"
#include "robot_adapter.h"

namespace magicbot_loco {

class MagicbotRealAdapter final : public RobotAdapter {
public:
    MagicbotRealAdapter(MagicbotSdkAdapter& robot, SdkRobotState& state)
        : robot_(robot),
          state_(state)
    {
    }

    const char* name() const override { return "magicbot-real"; }

    RobotSnapshot read_snapshot() override { return state_.snapshot(); }

    void write_target(const JointTarget& target) override
    {
        const auto snap = state_.snapshot();
        if (target.mode == JointTargetMode::ZeroTorque) {
            robot_.publish_damping(snap.counts, 0.0f);
            command_published_ = true;
            return;
        }
        if (target.mode == JointTargetMode::Damping) {
            robot_.publish_damping(snap.counts, target.damping_kd);
            command_published_ = true;
            return;
        }
        robot_.publish_sdk24_command(
            snap.counts,
            target.q,
            target.gains.kp,
            target.gains.kd,
            false,
            target.damping_kd);
        command_published_ = true;
    }

    void write_damping(float damping_kd) override
    {
        robot_.publish_damping(state_.snapshot().counts, damping_kd);
        command_published_ = true;
    }

    AdapterTelemetry telemetry() const override
    {
        AdapterTelemetry out;
        out.backend = name();
        out.state_age_ms = state_.state_age_ms();
        out.command_published = command_published_;
        return out;
    }

private:
    MagicbotSdkAdapter& robot_;
    SdkRobotState& state_;
    bool command_published_{false};
};

}  // namespace magicbot_loco
