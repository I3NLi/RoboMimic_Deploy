#pragma once

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

inline constexpr const char kBeyondMimicPolicyKey[] = "BeyondMimic";
// TrackMimic selects a BeyondMimic-trained, trajectory-conditioned config.
inline constexpr const char kTrackMimicPolicyKey[] = "TrackMimic";

struct ModeRequest {
    bool requested{false};
    ControlMode mode{ControlMode::Stand};
    std::string external_policy_key;

    static ModeRequest none() { return {}; }
    static ModeRequest enter(ControlMode next_mode) { return ModeRequest{true, next_mode, {}}; }
    static ModeRequest enter_external(ControlMode next_mode, std::string policy_key)
    {
        return ModeRequest{true, next_mode, std::move(policy_key)};
    }
};

struct ModeTransition {
    ControlMode previous{ControlMode::Stand};
    ControlMode current{ControlMode::Stand};
    bool changed{false};
    bool reset_policy_history{false};
    bool zero_command{false};
    bool seed_target_from_state{false};
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

inline bool is_external_policy_mode(ControlMode mode)
{
    return mode == ControlMode::Dance || mode == ControlMode::Skill;
}

inline bool is_policy_mode(ControlMode mode)
{
    return mode == ControlMode::Loco || is_external_policy_mode(mode);
}

inline ModeRequest mode_request_for_control_mode(ControlMode mode, std::string external_policy_key = {})
{
    if (!is_external_policy_mode(mode)) {
        return ModeRequest::enter(mode);
    }
    if (mode == ControlMode::Dance && external_policy_key.empty()) {
        external_policy_key = kBeyondMimicPolicyKey;
    } else if (mode == ControlMode::Skill && external_policy_key.empty()) {
        external_policy_key = kTrackMimicPolicyKey;
    }
    return ModeRequest::enter_external(mode, std::move(external_policy_key));
}

class ModeManager {
public:
    explicit ModeManager(ControlMode initial_mode = ControlMode::Stand)
        : mode_(initial_mode)
    {
        enabled_[mode_index(ControlMode::Passive)] = true;
        enabled_[mode_index(ControlMode::Stand)] = true;
        enabled_[mode_index(ControlMode::Loco)] = true;
        enabled_[mode_index(ControlMode::FinalDamping)] = true;
    }

    ControlMode mode() const { return mode_; }

    void set_enabled(ControlMode mode, bool enabled)
    {
        enabled_[mode_index(mode)] = enabled;
    }

    bool is_enabled(ControlMode mode) const
    {
        return enabled_[mode_index(mode)];
    }

    ModeTransition apply(ModeRequest request)
    {
        ModeTransition transition;
        transition.previous = mode_;
        transition.current = mode_;

        if (!request.requested || request.mode == mode_) {
            return transition;
        }

        validate_request(request.mode);
        transition.current = request.mode;
        transition.changed = true;
        transition.reset_policy_history = is_policy_mode(mode_) || is_policy_mode(request.mode);
        transition.zero_command = request.mode != ControlMode::Loco;
        transition.seed_target_from_state =
            request.mode == ControlMode::Passive || request.mode == ControlMode::FinalDamping;
        mode_ = request.mode;
        return transition;
    }

private:
    void validate_request(ControlMode requested) const
    {
        if (!is_enabled(requested)) {
            throw std::runtime_error(
                std::string("ModeManager mode is not enabled: ") +
                control_mode_name(requested));
        }
    }

    static constexpr int mode_index(ControlMode mode)
    {
        return static_cast<int>(mode);
    }

    ControlMode mode_{ControlMode::Stand};
    bool enabled_[6]{};
};

}  // namespace magicbot_loco
