#pragma once

#include "magicbot_loco_core.h"
#include "mode_manager.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace magicbot_loco {

struct Command {
    std::array<float, 3> velocity{0.0f, 0.0f, 0.0f};
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

    ControlMode mode() const { return mode_manager_.mode(); }

    const JointGains& gains() const { return gains_; }

    ControllerCoreOutput step(
        const RobotSnapshot& snapshot,
        const Command& command,
        ModeRequest request,
        float control_dt_s)
    {
        const ModeTransition transition = mode_manager_.apply(request);
        if (transition.reset_policy_history) {
            reset_policy();
        }
        if (transition.seed_target_from_state) {
            command_target_ = snapshot.q;
        }

        const float dt = std::max(control_dt_s, 1e-6f);
        policy_elapsed_s_ += dt;
        const ControlMode active_mode = mode_manager_.mode();

        if (active_mode == ControlMode::Passive || active_mode == ControlMode::FinalDamping) {
            projected_gravity_ = gravity_orientation(snapshot.quat);
            command_target_ = snapshot.q;
            return make_output(false);
        }
        if (active_mode == ControlMode::Stand) {
            return step_stand(snapshot, dt);
        }
        if (active_mode == ControlMode::Loco) {
            return step_loco(snapshot, transition.zero_command ? Command{} : command);
        }

        throw std::runtime_error(
            std::string("ControllerCore mode is not wired yet: ") + control_mode_name(active_mode));
    }

private:
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
        out.target.damping_only =
            mode_manager_.mode() == ControlMode::Passive || mode_manager_.mode() == ControlMode::FinalDamping;
        out.target.damping_kd = options_.damping_kd;
        out.telemetry.mode = mode_manager_.mode();
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
    ModeManager mode_manager_{ControlMode::Stand};
    float policy_elapsed_s_{kDefaultPolicyDt};
    bool have_previous_raw_policy_target_{false};
    int policy_steps_{0};
};

}  // namespace magicbot_loco
