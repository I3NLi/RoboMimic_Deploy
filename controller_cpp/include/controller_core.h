#pragma once

#include "magicbot_loco_core.h"
#include "mode_manager.h"
#include "policy_adapter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
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
    std::string external_policy;
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
        command_gains_ = gains_;
    }

    void warmup(int rounds) { loco_policy_.warmup(rounds); }

    void seed_target(const JointArray& target)
    {
        command_target_ = target;
        raw_policy_target_ = target;
        have_previous_raw_policy_target_ = false;
    }

    void set_default_target(const JointArray& target)
    {
        default_target_ = target;
        seed_target(target);
    }

    void set_gains(JointGains gains)
    {
        for (int i = 0; i < kNumJoints; ++i) {
            gains.kp[i] *= options_.kp_scale;
            gains.kd[i] *= options_.kd_scale;
        }
        gains_ = gains;
        command_gains_ = gains_;
    }

    void reset_policy()
    {
        loco_policy_.reset();
        policy_elapsed_s_ = cfg_.policy_dt;
        have_previous_raw_policy_target_ = false;
    }

    ControlMode mode() const { return mode_manager_.mode(); }

    const JointGains& gains() const { return gains_; }

    void register_external_policy(ExternalPolicyAdapter& policy)
    {
        register_external_policy(
            policy.name(),
            policy,
            external_policies_.find(policy.mode()) == external_policies_.end());
    }

    void register_external_policy(std::string key, ExternalPolicyAdapter& policy, bool make_default = false)
    {
        const ControlMode policy_mode = policy.mode();
        if (policy_mode != ControlMode::Dance && policy_mode != ControlMode::Skill) {
            throw std::runtime_error(
                std::string("external policy mode must be DANCE or SKILL, got ") +
                control_mode_name(policy_mode));
        }
        if (key.empty()) key = policy.name();
        keyed_external_policies_[std::make_pair(policy_mode, key)] = &policy;
        if (make_default || external_policies_[policy_mode] == nullptr) {
            external_policies_[policy_mode] = &policy;
        }
        mode_manager_.set_enabled(policy_mode, true);
    }

    ControllerCoreOutput step(
        const RobotSnapshot& snapshot,
        const Command& command,
        ModeRequest request,
        float control_dt_s)
    {
        ExternalPolicyAdapter* requested_external_policy = nullptr;
        if (request.requested && is_external_policy_mode(request.mode)) {
            requested_external_policy = resolve_external_policy(request.mode, request.external_policy_key);
        }

        const ModeTransition transition = mode_manager_.apply(request);
        bool external_policy_changed = false;
        if (requested_external_policy != nullptr) {
            external_policy_changed = requested_external_policy != active_external_policy_;
            active_external_policy_ = requested_external_policy;
            active_external_policy_mode_ = request.mode;
        }

        if (transition.reset_policy_history || external_policy_changed) {
            reset_policy();
        }
        if (transition.seed_target_from_state) {
            command_target_ = snapshot.q;
        }
        if (transition.reset_policy_history || external_policy_changed) {
            reset_external_policy(snapshot);
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
        if (active_mode == ControlMode::Dance || active_mode == ControlMode::Skill) {
            const bool zero_command = transition.zero_command || external_policy_changed;
            return step_external_policy(snapshot, zero_command ? Command{} : command);
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
        command_gains_ = gains_;
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
            command_gains_ = gains_;
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

    ControllerCoreOutput step_external_policy(const RobotSnapshot& snapshot, const Command& command)
    {
        ExternalPolicyAdapter* policy = external_policy(mode_manager_.mode());
        if (policy == nullptr) {
            throw std::runtime_error(
                std::string("no external policy registered for ") + control_mode_name(mode_manager_.mode()));
        }

        if (policy_elapsed_s_ + 1e-6f < cfg_.policy_dt) {
            projected_gravity_ = gravity_orientation(snapshot.quat);
            safety_.check(
                snapshot,
                &command_target_,
                have_previous_raw_policy_target_ ? &raw_policy_target_ : nullptr,
                nullptr);
            return make_output(false);
        }

        ExternalPolicyInput input;
        input.snapshot = snapshot;
        input.velocity_command = command.velocity;
        input.dt_s = cfg_.policy_dt;
        const ExternalPolicyOutput policy_output = policy->step(input);
        JointGains policy_gains = gains_;
        if (policy_output.override_gains) {
            policy_gains.kp = policy_output.kp_motor;
            policy_gains.kd = policy_output.kd_motor;
            bool has_tau_limit_override = false;
            for (float limit : policy_output.tau_limit_motor) {
                if (limit > 0.0f) {
                    has_tau_limit_override = true;
                    break;
                }
            }
            if (has_tau_limit_override) {
                policy_gains.tau_limit = policy_output.tau_limit_motor;
            }
            for (int i = 0; i < kNumJoints; ++i) {
                policy_gains.kp[i] *= options_.kp_scale;
                policy_gains.kd[i] *= options_.kd_scale;
            }
        }

        safety_.check(
            snapshot,
            nullptr,
            &policy_output.target_motor,
            have_previous_raw_policy_target_ ? &raw_policy_target_ : nullptr);
        const auto limited = torque_limited_target(
            policy_output.target_motor,
            snapshot.q,
            snapshot.dq,
            policy_gains.kp,
            policy_gains.kd,
            policy_gains.tau_limit,
            cfg_.tau_limit_scale);
        command_target_ = clamp_and_rate_limit(
            limited,
            command_target_,
            options_.max_target_rate,
            cfg_.policy_dt,
            options_.joint_limit_margin);
        command_gains_ = policy_gains;
        safety_.check(
            snapshot,
            &command_target_,
            &policy_output.target_motor,
            have_previous_raw_policy_target_ ? &raw_policy_target_ : nullptr);

        raw_policy_target_ = policy_output.target_motor;
        have_previous_raw_policy_target_ = true;
        projected_gravity_ = gravity_orientation(snapshot.quat);
        policy_elapsed_s_ = 0.0f;
        ++policy_steps_;

        if (policy_output.complete) {
            const ModeTransition transition = mode_manager_.apply(ModeRequest::enter(policy_output.next_mode));
            if (transition.reset_policy_history) {
                reset_policy();
                reset_external_policy(snapshot);
            }
            if (transition.seed_target_from_state) {
                command_target_ = snapshot.q;
            }
        }
        return make_output(true);
    }

    ControllerCoreOutput make_output(bool policy_evaluated) const
    {
        ControllerCoreOutput out;
        out.target.q = command_target_;
        out.target.gains = command_gains_;
        out.target.damping_only =
            mode_manager_.mode() == ControlMode::Passive || mode_manager_.mode() == ControlMode::FinalDamping;
        out.target.damping_kd = options_.damping_kd;
        out.telemetry.mode = mode_manager_.mode();
        if (is_external_policy_mode(mode_manager_.mode())) {
            ExternalPolicyAdapter* policy = external_policy(mode_manager_.mode());
            if (policy != nullptr) out.telemetry.external_policy = policy->name();
        }
        out.telemetry.policy_evaluated = policy_evaluated;
        out.telemetry.policy_steps = policy_steps_;
        out.telemetry.raw_policy_target = raw_policy_target_;
        out.telemetry.command_target = command_target_;
        out.telemetry.projected_gravity = projected_gravity_;
        return out;
    }

    ExternalPolicyAdapter* external_policy(ControlMode mode) const
    {
        if (active_external_policy_ != nullptr && active_external_policy_mode_ == mode) {
            return active_external_policy_;
        }
        return default_external_policy(mode);
    }

    ExternalPolicyAdapter* default_external_policy(ControlMode mode) const
    {
        const auto it = external_policies_.find(mode);
        return it == external_policies_.end() ? nullptr : it->second;
    }

    ExternalPolicyAdapter* resolve_external_policy(ControlMode mode, const std::string& key) const
    {
        if (!key.empty()) {
            const auto it = keyed_external_policies_.find(std::make_pair(mode, key));
            if (it == keyed_external_policies_.end()) {
                throw std::runtime_error(
                    std::string("no external policy registered for ") + control_mode_name(mode) +
                    " key=" + key);
            }
            return it->second;
        }
        ExternalPolicyAdapter* policy = default_external_policy(mode);
        if (policy == nullptr) {
            throw std::runtime_error(
                std::string("no external policy registered for ") + control_mode_name(mode));
        }
        return policy;
    }

    void reset_external_policy(const RobotSnapshot& snapshot)
    {
        ExternalPolicyAdapter* policy = external_policy(mode_manager_.mode());
        if (policy != nullptr) {
            policy->reset(snapshot);
        }
    }

    LocoConfig cfg_;
    ControllerCoreOptions options_;
    OnnxLocoPolicy loco_policy_;
    MotionSafety safety_;
    JointGains gains_{};
    JointGains command_gains_{};
    JointArray default_target_{};
    JointArray command_target_{};
    JointArray raw_policy_target_{};
    std::array<float, 3> projected_gravity_{0.0f, 0.0f, -1.0f};
    ModeManager mode_manager_{ControlMode::Stand};
    std::map<ControlMode, ExternalPolicyAdapter*> external_policies_;
    std::map<std::pair<ControlMode, std::string>, ExternalPolicyAdapter*> keyed_external_policies_;
    ExternalPolicyAdapter* active_external_policy_{nullptr};
    ControlMode active_external_policy_mode_{ControlMode::Stand};
    float policy_elapsed_s_{kDefaultPolicyDt};
    bool have_previous_raw_policy_target_{false};
    int policy_steps_{0};
};

}  // namespace magicbot_loco
