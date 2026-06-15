#pragma once

#include "controller_core.h"
#include "fsm_external_policy_adapter.h"
#include "native_fsm_policy_types.h"
#include "beyond_mimic_policy.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace magicbot_loco {

class NativeBeyondMimicExternalPolicyRegistry {
public:
    explicit NativeBeyondMimicExternalPolicyRegistry(float policy_dt)
        : policy_dt_(policy_dt),
          dance_state_(kNumJoints),
          dance_output_(kNumJoints)
    {
    }

    void register_dance(ControllerCore& core, const std::string& yaml_path)
    {
        dance_policy_ = std::make_unique<::BeyondMimicPolicy>(
            dance_state_,
            dance_output_,
            yaml_path,
            policy_dt_);
        dance_adapter_ = std::make_unique<BeyondAdapter>(
            ControlMode::Dance,
            kBeyondMimicPolicyKey,
            ::FSMStateName::SKILL_BEYOND_MIMIC,
            dance_state_,
            dance_output_,
            *dance_policy_,
            ::control_mode_for_fsm_state);
        core.register_external_policy(kBeyondMimicPolicyKey, *dance_adapter_, true);
    }

    void register_track_mimic(ControllerCore& core, const std::string& yaml_path)
    {
        register_trajectory_variant(core, kTrackMimicPolicyKey, yaml_path, "TrackMimic", true);
    }

    void register_trajectory_variant(
        ControllerCore& core,
        std::string policy_key,
        const std::string& yaml_path,
        std::string state_label = {},
        bool make_default = false)
    {
        if (policy_key.empty()) {
            throw std::invalid_argument("BeyondMimic trajectory policy key must not be empty");
        }
        if (state_label.empty()) {
            state_label = policy_key;
        }

        auto variant = std::make_unique<TrajectoryVariant>();
        // TrackMimic and future trajectory-following skills are still
        // BeyondMimicPolicy instances. The required motion_file selects the
        // reference trajectory; it does not select a separate policy family.
        variant->policy = std::make_unique<::BeyondMimicPolicy>(
            variant->state,
            variant->output,
            yaml_path,
            policy_dt_,
            ::FSMStateName::SKILL_TRACK_MIMIC,
            std::move(state_label),
            /*require_motion_file=*/true);
        variant->adapter = std::make_unique<BeyondAdapter>(
            ControlMode::Skill,
            policy_key,
            ::FSMStateName::SKILL_TRACK_MIMIC,
            variant->state,
            variant->output,
            *variant->policy,
            ::control_mode_for_fsm_state);
        core.register_external_policy(policy_key, *variant->adapter, make_default || trajectory_variants_.empty());
        trajectory_variants_.push_back(std::move(variant));
    }

private:
    using BeyondAdapter =
        FsmExternalPolicyAdapter<::BeyondMimicPolicy, ::StateAndCmd, ::PolicyOutput, ::FSMStateName>;

    struct TrajectoryVariant {
        TrajectoryVariant()
            : state(kNumJoints),
              output(kNumJoints)
        {
        }

        ::StateAndCmd state;
        ::PolicyOutput output;
        std::unique_ptr<::BeyondMimicPolicy> policy;
        std::unique_ptr<BeyondAdapter> adapter;
    };

    float policy_dt_{0.02f};
    ::StateAndCmd dance_state_;
    ::PolicyOutput dance_output_;
    std::unique_ptr<::BeyondMimicPolicy> dance_policy_;
    std::unique_ptr<BeyondAdapter> dance_adapter_;
    std::vector<std::unique_ptr<TrajectoryVariant>> trajectory_variants_;
};

}  // namespace magicbot_loco
