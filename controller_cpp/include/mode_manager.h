#pragma once

#include <stdexcept>
#include <string>

namespace magicbot_loco {

enum class ControlMode {
    Passive,
    Stand,
    Loco,
    Dance,
    Skill,
    FinalDamping,
};

struct ModeRequest {
    bool requested{false};
    ControlMode mode{ControlMode::Stand};

    static ModeRequest none() { return {}; }
    static ModeRequest enter(ControlMode next_mode) { return ModeRequest{true, next_mode}; }
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

inline bool is_policy_mode(ControlMode mode)
{
    return mode == ControlMode::Loco || mode == ControlMode::Dance || mode == ControlMode::Skill;
}

class ModeManager {
public:
    explicit ModeManager(ControlMode initial_mode = ControlMode::Stand)
        : mode_(initial_mode)
    {
    }

    ControlMode mode() const { return mode_; }

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
    static void validate_request(ControlMode requested)
    {
        if (requested == ControlMode::Dance || requested == ControlMode::Skill) {
            throw std::runtime_error(
                std::string("ModeManager requires a skill policy adapter before entering ") +
                control_mode_name(requested));
        }
    }

    ControlMode mode_{ControlMode::Stand};
};

}  // namespace magicbot_loco
