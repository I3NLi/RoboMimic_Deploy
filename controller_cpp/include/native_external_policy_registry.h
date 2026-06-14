#pragma once

#include "controller_core.h"
#include "fsm_external_policy_adapter.h"
#include "native_fsm_policy_types.h"
#include "beyond_mimic_policy.h"

#include <memory>
#include <string>

namespace magicbot_loco {

class NativeBeyondMimicExternalPolicyRegistry {
public:
    explicit NativeBeyondMimicExternalPolicyRegistry(float policy_dt)
        : policy_dt_(policy_dt),
          dance_state_(kNumJoints),
          dance_output_(kNumJoints),
          track_state_(kNumJoints),
          track_output_(kNumJoints)
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
        // TrackMimic is a keyed BeyondMimic trajectory-conditioned config: the
        // implementation is still BeyondMimicPolicy, with an extra trajectory.
        track_policy_ = std::make_unique<::BeyondMimicPolicy>(
            track_state_,
            track_output_,
            yaml_path,
            policy_dt_,
            ::FSMStateName::SKILL_TRACK_MIMIC,
            "TrackMimic",
            false);
        track_adapter_ = std::make_unique<BeyondAdapter>(
            ControlMode::Skill,
            kTrackMimicPolicyKey,
            ::FSMStateName::SKILL_TRACK_MIMIC,
            track_state_,
            track_output_,
            *track_policy_,
            ::control_mode_for_fsm_state);
        core.register_external_policy(kTrackMimicPolicyKey, *track_adapter_, true);
    }

private:
    using BeyondAdapter =
        FsmExternalPolicyAdapter<::BeyondMimicPolicy, ::StateAndCmd, ::PolicyOutput, ::FSMStateName>;

    float policy_dt_{0.02f};
    ::StateAndCmd dance_state_;
    ::PolicyOutput dance_output_;
    ::StateAndCmd track_state_;
    ::PolicyOutput track_output_;
    std::unique_ptr<::BeyondMimicPolicy> dance_policy_;
    std::unique_ptr<::BeyondMimicPolicy> track_policy_;
    std::unique_ptr<BeyondAdapter> dance_adapter_;
    std::unique_ptr<BeyondAdapter> track_adapter_;
};

}  // namespace magicbot_loco
