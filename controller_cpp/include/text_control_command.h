#pragma once

#include "mode_manager.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace magicbot_loco {

enum class TextControlAction {
    Loco,
    WalkForward,
    RunForward,
    Stand,
    ResetStand,
    Passive,
    Dance,
    Skill,
    FinalDamping,
    Zero,
    Pause,
    Resume,
    Stop,
    ToggleLoco,
};

struct TextControlActionEffect {
    bool mode_requested{false};
    ControlMode mode{ControlMode::Stand};
    bool zero_command{false};
    bool command_requested{false};
    std::array<float, 3> command{0.0f, 0.0f, 0.0f};
    bool unpause{false};
    bool pause{false};
    bool stop{false};
    bool toggle_loco{false};
    bool reset_stand{false};
    std::string external_policy_key;
};

struct TextControlIntentState {
    std::array<float, 3> command{0.0f, 0.0f, 0.0f};
    ControlMode desired_mode{ControlMode::Stand};
    std::string desired_external_policy_key;
    bool paused{false};
    bool running{true};
    bool reset_requested{false};
};

struct TextControlIntentOptions {
    bool dance_enabled{true};
    bool skill_enabled{true};
};

inline constexpr float kTextControlWalkVx = 0.25f;
inline constexpr float kTextControlRunVx = 0.65f;

inline std::array<float, 3> command_for_control_mode(std::array<float, 3> command, ControlMode mode)
{
    if (mode != ControlMode::Loco) {
        return {0.0f, 0.0f, 0.0f};
    }
    return command;
}

enum class TextControlIntentRejectReason {
    None,
    DanceDisabled,
    SkillDisabled,
};

struct TextControlIntentApplyResult {
    bool applied{true};
    TextControlIntentRejectReason reject_reason{TextControlIntentRejectReason::None};
};

struct TextControlOperation {
    enum class Type {
        Velocity,
        Action,
    };

    Type type{Type::Action};
    int axis{-1};
    float value{0.0f};
    TextControlAction action{TextControlAction::Zero};
    std::string external_policy_key;

    static TextControlOperation velocity(int axis_index, float axis_value)
    {
        TextControlOperation op;
        op.type = Type::Velocity;
        op.axis = axis_index;
        op.value = axis_value;
        return op;
    }

    static TextControlOperation action_op(TextControlAction next_action, std::string policy_key = {})
    {
        TextControlOperation op;
        op.type = Type::Action;
        op.action = next_action;
        op.external_policy_key = std::move(policy_key);
        return op;
    }
};

inline float clamp_text_control_value(float value)
{
    return std::clamp(value, -1.0f, 1.0f);
}

inline std::string lower_ascii_copy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

inline bool parse_text_control_float(const std::string& token, float& value)
{
    char* end = nullptr;
    errno = 0;
    const float parsed = std::strtof(token.c_str(), &end);
    if (end == token.c_str() || *end != '\0' || errno == ERANGE || !std::isfinite(parsed)) {
        return false;
    }
    value = clamp_text_control_value(parsed);
    return true;
}

inline bool text_control_action_from_word(const std::string& word, TextControlAction& action)
{
    if (word == "loco" || word == "run") {
        action = TextControlAction::Loco;
        return true;
    }
    if (word == "walk" || word == "walk_forward") {
        action = TextControlAction::WalkForward;
        return true;
    }
    if (word == "run_forward" || word == "sprint" || word == "fast") {
        action = TextControlAction::RunForward;
        return true;
    }
    if (word == "stand") {
        action = TextControlAction::Stand;
        return true;
    }
    if (word == "reset" || word == "restand") {
        action = TextControlAction::ResetStand;
        return true;
    }
    if (word == "passive" || word == "damping") {
        action = TextControlAction::Passive;
        return true;
    }
    if (word == "dance" || word == "beyond" || word == "beyondmimic") {
        action = TextControlAction::Dance;
        return true;
    }
    if (word == "skill" || word == "track" || word == "trackmimic" || word == "track_mimic") {
        action = TextControlAction::Skill;
        return true;
    }
    if (word == "final" || word == "finaldamping" || word == "final_damping" ||
        word == "fail_safe" || word == "failsafe") {
        action = TextControlAction::FinalDamping;
        return true;
    }
    if (word == "zero" || word == "x") {
        action = TextControlAction::Zero;
        return true;
    }
    if (word == "pause") {
        action = TextControlAction::Pause;
        return true;
    }
    if (word == "resume") {
        action = TextControlAction::Resume;
        return true;
    }
    if (word == "stop" || word == "exit") {
        action = TextControlAction::Stop;
        return true;
    }
    if (word == "toggle") {
        action = TextControlAction::ToggleLoco;
        return true;
    }
    return false;
}

inline TextControlActionEffect text_control_action_effect(
    TextControlAction action,
    std::string external_policy_key = {})
{
    TextControlActionEffect effect;
    switch (action) {
    case TextControlAction::Loco:
        effect.mode_requested = true;
        effect.mode = ControlMode::Loco;
        effect.unpause = true;
        break;
    case TextControlAction::WalkForward:
        effect.mode_requested = true;
        effect.mode = ControlMode::Loco;
        effect.command_requested = true;
        effect.command = {kTextControlWalkVx, 0.0f, 0.0f};
        effect.unpause = true;
        break;
    case TextControlAction::RunForward:
        effect.mode_requested = true;
        effect.mode = ControlMode::Loco;
        effect.command_requested = true;
        effect.command = {kTextControlRunVx, 0.0f, 0.0f};
        effect.unpause = true;
        break;
    case TextControlAction::Stand:
        effect.mode_requested = true;
        effect.mode = ControlMode::Stand;
        effect.zero_command = true;
        effect.unpause = true;
        break;
    case TextControlAction::ResetStand:
        effect.mode_requested = true;
        effect.mode = ControlMode::Stand;
        effect.zero_command = true;
        effect.unpause = true;
        effect.reset_stand = true;
        break;
    case TextControlAction::Passive:
        effect.mode_requested = true;
        effect.mode = ControlMode::Passive;
        effect.zero_command = true;
        effect.unpause = true;
        break;
    case TextControlAction::Dance:
        effect.mode_requested = true;
        effect.mode = ControlMode::Dance;
        effect.external_policy_key = kBeyondMimicPolicyKey;
        effect.zero_command = true;
        effect.unpause = true;
        break;
    case TextControlAction::Skill:
        effect.mode_requested = true;
        effect.mode = ControlMode::Skill;
        effect.external_policy_key =
            external_policy_key.empty() ? kTrackMimicPolicyKey : std::move(external_policy_key);
        effect.zero_command = true;
        effect.unpause = true;
        break;
    case TextControlAction::FinalDamping:
        effect.mode_requested = true;
        effect.mode = ControlMode::FinalDamping;
        effect.zero_command = true;
        effect.unpause = true;
        break;
    case TextControlAction::Zero:
        effect.zero_command = true;
        break;
    case TextControlAction::Pause:
        effect.pause = true;
        break;
    case TextControlAction::Resume:
        effect.unpause = true;
        break;
    case TextControlAction::Stop:
        effect.stop = true;
        break;
    case TextControlAction::ToggleLoco:
        effect.toggle_loco = true;
        effect.unpause = true;
        break;
    }
    return effect;
}

inline ModeRequest mode_request_for_text_control_effect(const TextControlActionEffect& effect)
{
    if (!effect.mode_requested) {
        return ModeRequest::none();
    }
    return mode_request_for_control_mode(effect.mode, effect.external_policy_key);
}

inline TextControlIntentApplyResult apply_text_control_effect_to_intent(
    TextControlIntentState& intent,
    const TextControlActionEffect& effect,
    const TextControlIntentOptions& options = TextControlIntentOptions{})
{
    if (effect.mode_requested && effect.mode == ControlMode::Dance && !options.dance_enabled) {
        return {false, TextControlIntentRejectReason::DanceDisabled};
    }
    if (effect.mode_requested && effect.mode == ControlMode::Skill && !options.skill_enabled) {
        return {false, TextControlIntentRejectReason::SkillDisabled};
    }

    if (effect.zero_command) {
        intent.command = {0.0f, 0.0f, 0.0f};
    }
    if (effect.command_requested) {
        intent.command = effect.command;
    }
    if (effect.pause) {
        intent.paused = true;
    }
    if (effect.unpause) {
        intent.paused = false;
    }
    if (effect.stop) {
        intent.running = false;
    }
    if (effect.reset_stand) {
        const ModeRequest reset_request = mode_request_for_text_control_effect(effect);
        if (reset_request.requested) {
            intent.desired_mode = reset_request.mode;
            intent.desired_external_policy_key = reset_request.external_policy_key;
        }
        intent.reset_requested = true;
        return {};
    }
    if (effect.toggle_loco) {
        const ModeRequest toggle_request = mode_request_for_loco_toggle(intent.desired_mode);
        if (toggle_request.mode == ControlMode::Stand) {
            intent.command = {0.0f, 0.0f, 0.0f};
        } else {
            intent.reset_requested = true;
        }
        intent.desired_mode = toggle_request.mode;
        intent.desired_external_policy_key = toggle_request.external_policy_key;
    }
    if (!effect.mode_requested) {
        return {};
    }

    const ModeRequest mode_request = mode_request_for_text_control_effect(effect);
    if (mode_request.mode == ControlMode::Loco && intent.desired_mode != ControlMode::Loco) {
        intent.reset_requested = true;
    }
    if (mode_request.mode == ControlMode::Stand) {
        intent.reset_requested = true;
    }
    intent.desired_mode = mode_request.mode;
    intent.desired_external_policy_key = mode_request.external_policy_key;
    return {};
}

inline TextControlIntentApplyResult apply_text_control_action_to_intent(
    TextControlIntentState& intent,
    TextControlAction action,
    const TextControlIntentOptions& options = TextControlIntentOptions{},
    std::string external_policy_key = {})
{
    return apply_text_control_effect_to_intent(
        intent,
        text_control_action_effect(action, std::move(external_policy_key)),
        options);
}

inline bool parse_text_control_action_token(
    const std::string& raw_token,
    TextControlAction& action,
    std::string& external_policy_key)
{
    const size_t sep = raw_token.find(':');
    const std::string action_word = sep == std::string::npos ? raw_token : raw_token.substr(0, sep);
    if (!text_control_action_from_word(lower_ascii_copy(action_word), action)) {
        return false;
    }
    external_policy_key = sep == std::string::npos ? std::string() : raw_token.substr(sep + 1);
    if (action != TextControlAction::Skill) {
        external_policy_key.clear();
    }
    return true;
}

inline std::vector<TextControlOperation> parse_text_control_operations(std::string message)
{
    for (char& ch : message) {
        if (ch == ',' || ch == ';' || ch == '\n' || ch == '\r' || ch == '\t') ch = ' ';
    }

    std::vector<TextControlOperation> operations;
    int numeric_index = 0;
    std::istringstream stream(message);
    std::string token;
    while (stream >> token) {
        const auto eq = token.find('=');
        if (eq != std::string::npos) {
            const std::string key = lower_ascii_copy(token.substr(0, eq));
            const std::string raw_value = token.substr(eq + 1);
            const std::string value = lower_ascii_copy(raw_value);
            float parsed = 0.0f;
            if ((key == "vx" || key == "x") && parse_text_control_float(value, parsed)) {
                operations.push_back(TextControlOperation::velocity(0, parsed));
            } else if ((key == "vy" || key == "y") && parse_text_control_float(value, parsed)) {
                operations.push_back(TextControlOperation::velocity(1, parsed));
            } else if ((key == "wz" || key == "yaw") && parse_text_control_float(value, parsed)) {
                operations.push_back(TextControlOperation::velocity(2, parsed));
            } else if (key == "mode") {
                TextControlAction action{};
                std::string external_policy_key;
                if (parse_text_control_action_token(raw_value, action, external_policy_key)) {
                    operations.push_back(
                        TextControlOperation::action_op(action, std::move(external_policy_key)));
                }
            }
            continue;
        }

        float parsed = 0.0f;
        if (numeric_index < 3 && parse_text_control_float(token, parsed)) {
            operations.push_back(TextControlOperation::velocity(numeric_index, parsed));
            ++numeric_index;
            continue;
        }

        TextControlAction action{};
        std::string external_policy_key;
        if (parse_text_control_action_token(token, action, external_policy_key)) {
            operations.push_back(TextControlOperation::action_op(action, std::move(external_policy_key)));
        }
    }

    return operations;
}

}  // namespace magicbot_loco
