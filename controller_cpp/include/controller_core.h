#pragma once

#include "magicbot_loco_core.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace magicbot_loco {

enum class ControlMode {
    Passive,
    Stand,
    Loco,
    Dance,
    Skill,
    FinalDamping,
};

struct Command {
    std::array<float, 3> velocity{0.0f, 0.0f, 0.0f};
};

struct ModeRequest {
    bool requested{false};
    ControlMode mode{ControlMode::Stand};

    static ModeRequest none() { return {}; }
    static ModeRequest enter(ControlMode next_mode) { return ModeRequest{true, next_mode}; }
};

struct JointGains {
    JointArray kp{};
    JointArray kd{};
    JointArray tau_limit{};
};

struct JointTarget {
    JointArray q{};
    JointGains gains{};
    bool damping_only{false};
    float damping_kd{3.0f};
};

struct ControllerTelemetry {
    ControlMode mode{ControlMode::Stand};
    bool policy_evaluated{false};
    int policy_steps{0};
    JointArray raw_policy_target{};
    JointArray command_target{};
    std::array<float, 3> projected_gravity{0.0f, 0.0f, -1.0f};
};

struct ControllerCoreOutput {
    JointTarget target{};
    ControllerTelemetry telemetry{};
};

struct ControllerCoreOptions {
    SafetyConfig safety{};
    float kp_scale{1.0f};
    float kd_scale{1.0f};
    float max_target_rate{4.0f};
    float joint_limit_margin{0.01f};
    float damping_kd{3.0f};
};

inline const char* control_mode_name(ControlMode mode)
{
    switch (mode) {
    case ControlMode::Passive:
        return "PASSIVE";
    case ControlMode::Stand:
        return "STAND";
    case ControlMode::Loco:
        return "LOCO";
    case ControlMode::Dance:
        return "DANCE";
    case ControlMode::Skill:
        return "SKILL";
    case ControlMode::FinalDamping:
        return "FINAL_DAMPING";
    }
    return "UNKNOWN";
}

class ControllerCore {
public:
    explicit ControllerCore(LocoConfig cfg, ControllerCoreOptions options = {})
        : cfg_(std::move(cfg)),
          options_(std::move(options)),
          loco_policy_(cfg_),
          safety_(options_.safety, cfg_),
          default_target_(cfg_.default_motor()),
          command_target_(default_target_),
          raw_policy_target_(default_target_),
          policy_elapsed_s_(cfg_.policy_dt)
    {
        gains_.kp = cfg_.kps_motor();
        gains_.kd = cfg_.kds_motor();
        gains_.tau_limit = cfg_.tau_limit_motor();
        for (int i = 0; i < kNumJoints; ++i) {
            gains_.kp[i] *= options_.kp_scale;
            gains_.kd[i] *= options_.kd_scale;
        }
    }

    void warmup(int rounds) { loco_policy_.warmup(rounds); }

    void seed_target(const JointArray& target)
    {
        command_target_ = target;
        raw_policy_target_ = target;
        have_previous_raw_policy_target_ = false;
    }

    void reset_policy()
    {
        loco_policy_.reset();
        policy_elapsed_s_ = cfg_.policy_dt;
        have_previous_raw_policy_target_ = false;
    }

    ControlMode mode() const { return mode_; }

    const JointGains& gains() const { return gains_; }

    ControllerCoreOutput step(
        const RobotSnapshot& snapshot,
        const Command& command,
        ModeRequest request,
        float control_dt_s)
    {
        if (request.requested) {
            switch_mode(request.mode, snapshot);
        }

        const float dt = std::max(control_dt_s, 1e-6f);
        policy_elapsed_s_ += dt;

        if (mode_ == ControlMode::Passive || mode_ == ControlMode::FinalDamping) {
            projected_gravity_ = gravity_orientation(snapshot.quat);
            command_target_ = snapshot.q;
            return make_output(false);
        }
        if (mode_ == ControlMode::Stand) {
            return step_stand(snapshot, dt);
        }
        if (mode_ == ControlMode::Loco) {
            return step_loco(snapshot, command);
        }

        throw std::runtime_error(
            std::string("ControllerCore mode is not wired yet: ") + control_mode_name(mode_));
    }

private:
    void switch_mode(ControlMode next_mode, const RobotSnapshot& snapshot)
    {
        if (next_mode == mode_) return;
        if (next_mode == ControlMode::Dance || next_mode == ControlMode::Skill) {
            throw std::runtime_error(
                std::string("ControllerCore mode requires a skill policy adapter: ") +
                control_mode_name(next_mode));
        }
        mode_ = next_mode;
        reset_policy();
        if (mode_ == ControlMode::Passive || mode_ == ControlMode::FinalDamping) {
            command_target_ = snapshot.q;
        }
    }

    ControllerCoreOutput step_stand(const RobotSnapshot& snapshot, float dt)
    {
        projected_gravity_ = gravity_orientation(snapshot.quat);
        const auto limited = torque_limited_target(
            default_target_,
            snapshot.q,
            snapshot.dq,
            gains_.kp,
            gains_.kd,
            gains_.tau_limit,
            cfg_.tau_limit_scale);
        command_target_ = clamp_and_rate_limit(
            limited,
            command_target_,
            options_.max_target_rate,
            dt,
            options_.joint_limit_margin);
        safety_.check(snapshot, &command_target_, &default_target_, nullptr);
        raw_policy_target_ = default_target_;
        return make_output(false);
    }

    ControllerCoreOutput step_loco(const RobotSnapshot& snapshot, const Command& command)
    {
        if (policy_elapsed_s_ + 1e-6f >= cfg_.policy_dt) {
            const auto gravity = gravity_orientation(snapshot.quat);
            projected_gravity_ = gravity;
            const auto infer =
                loco_policy_.infer(snapshot.q, snapshot.dq, snapshot.ang_vel, gravity, command.velocity);
            safety_.check(
                snapshot,
                nullptr,
                &infer.target_motor,
                have_previous_raw_policy_target_ ? &raw_policy_target_ : nullptr);
            const auto limited = torque_limited_target(
                infer.target_motor,
                snapshot.q,
                snapshot.dq,
                gains_.kp,
                gains_.kd,
                gains_.tau_limit,
                cfg_.tau_limit_scale);
            command_target_ = clamp_and_rate_limit(
                limited,
                command_target_,
                options_.max_target_rate,
                cfg_.policy_dt,
                options_.joint_limit_margin);
            safety_.check(
                snapshot,
                &command_target_,
                &infer.target_motor,
                have_previous_raw_policy_target_ ? &raw_policy_target_ : nullptr);
            raw_policy_target_ = infer.target_motor;
            have_previous_raw_policy_target_ = true;
            policy_elapsed_s_ = 0.0f;
            ++policy_steps_;
            return make_output(true);
        }

        projected_gravity_ = gravity_orientation(snapshot.quat);
        safety_.check(
            snapshot,
            &command_target_,
            have_previous_raw_policy_target_ ? &raw_policy_target_ : nullptr,
            nullptr);
        return make_output(false);
    }

    ControllerCoreOutput make_output(bool policy_evaluated) const
    {
        ControllerCoreOutput out;
        out.target.q = command_target_;
        out.target.gains = gains_;
        out.target.damping_only = mode_ == ControlMode::Passive || mode_ == ControlMode::FinalDamping;
        out.target.damping_kd = options_.damping_kd;
        out.telemetry.mode = mode_;
        out.telemetry.policy_evaluated = policy_evaluated;
        out.telemetry.policy_steps = policy_steps_;
        out.telemetry.raw_policy_target = raw_policy_target_;
        out.telemetry.command_target = command_target_;
        out.telemetry.projected_gravity = projected_gravity_;
        return out;
    }

    LocoConfig cfg_;
    ControllerCoreOptions options_;
    OnnxLocoPolicy loco_policy_;
    MotionSafety safety_;
    JointGains gains_{};
    JointArray default_target_{};
    JointArray command_target_{};
    JointArray raw_policy_target_{};
    std::array<float, 3> projected_gravity_{0.0f, 0.0f, -1.0f};
    ControlMode mode_{ControlMode::Stand};
    float policy_elapsed_s_{kDefaultPolicyDt};
    bool have_previous_raw_policy_target_{false};
    int policy_steps_{0};
};

}  // namespace magicbot_loco
