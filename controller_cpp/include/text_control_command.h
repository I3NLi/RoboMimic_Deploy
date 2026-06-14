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
#include <vector>

namespace magicbot_loco {

enum class TextControlAction {
    Loco,
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
    bool unpause{false};
    bool pause{false};
    bool stop{false};
    bool toggle_loco{false};
    bool reset_stand{false};
    std::string external_policy_key;
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

    static TextControlOperation velocity(int axis_index, float axis_value)
    {
        TextControlOperation op;
        op.type = Type::Velocity;
        op.axis = axis_index;
        op.value = axis_value;
        return op;
    }

    static TextControlOperation action_op(TextControlAction next_action)
    {
        TextControlOperation op;
        op.type = Type::Action;
        op.action = next_action;
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

inline TextControlActionEffect text_control_action_effect(TextControlAction action)
{
    TextControlActionEffect effect;
    switch (action) {
    case TextControlAction::Loco:
        effect.mode_requested = true;
        effect.mode = ControlMode::Loco;
        effect.unpause = true;
        break;
    case TextControlAction::Stand:
        effect.mode_requested = true;
        effect.mode = ControlMode::Stand;
        effect.zero_command = true;
        effect.unpause = true;
        break;
    case TextControlAction::ResetStand:
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
        effect.external_policy_key = kTrackMimicPolicyKey;
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
            const std::string value = lower_ascii_copy(token.substr(eq + 1));
            float parsed = 0.0f;
            if ((key == "vx" || key == "x") && parse_text_control_float(value, parsed)) {
                operations.push_back(TextControlOperation::velocity(0, parsed));
            } else if ((key == "vy" || key == "y") && parse_text_control_float(value, parsed)) {
                operations.push_back(TextControlOperation::velocity(1, parsed));
            } else if ((key == "wz" || key == "yaw") && parse_text_control_float(value, parsed)) {
                operations.push_back(TextControlOperation::velocity(2, parsed));
            } else if (key == "mode") {
                TextControlAction action{};
                if (text_control_action_from_word(value, action)) {
                    operations.push_back(TextControlOperation::action_op(action));
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
        if (text_control_action_from_word(lower_ascii_copy(token), action)) {
            operations.push_back(TextControlOperation::action_op(action));
        }
    }

    return operations;
}

}  // namespace magicbot_loco
