#include "text_control_command.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ml = magicbot_loco;

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool same_float(float lhs, float rhs)
{
    return std::fabs(lhs - rhs) < 1e-6f;
}

void expect_velocity(const ml::TextControlOperation& op, int axis, float value)
{
    require(op.type == ml::TextControlOperation::Type::Velocity, "expected velocity op");
    require(op.axis == axis, "unexpected velocity axis");
    require(same_float(op.value, value), "unexpected velocity value");
}

void expect_action(const ml::TextControlOperation& op, ml::TextControlAction action)
{
    require(op.type == ml::TextControlOperation::Type::Action, "expected action op");
    require(op.action == action, "unexpected action");
}

void check_named_velocity_and_mode()
{
    const auto ops = ml::parse_text_control_operations("VX=2.0 vy=-2.0 wz=0.125 mode=LoCo");
    require(ops.size() == 4, "named velocity and mode op count");
    expect_velocity(ops[0], 0, 1.0f);
    expect_velocity(ops[1], 1, -1.0f);
    expect_velocity(ops[2], 2, 0.125f);
    expect_action(ops[3], ml::TextControlAction::Loco);
}

void check_numeric_tokens_and_clear_modes()
{
    const auto ops = ml::parse_text_control_operations("0.5, -0.25; 0.1 passive final_damping");
    require(ops.size() == 5, "numeric tokens op count");
    expect_velocity(ops[0], 0, 0.5f);
    expect_velocity(ops[1], 1, -0.25f);
    expect_velocity(ops[2], 2, 0.1f);
    expect_action(ops[3], ml::TextControlAction::Passive);
    expect_action(ops[4], ml::TextControlAction::FinalDamping);
}

void check_aliases()
{
    ml::TextControlAction action{};
    require(ml::text_control_action_from_word("run", action), "run alias");
    require(action == ml::TextControlAction::Loco, "run should map to loco");
    require(ml::text_control_action_from_word("damping", action), "damping alias");
    require(action == ml::TextControlAction::Passive, "damping should map to passive");
    require(ml::text_control_action_from_word("beyondmimic", action), "beyondmimic alias");
    require(action == ml::TextControlAction::Dance, "beyondmimic should map to dance");
    require(ml::text_control_action_from_word("track_mimic", action), "track_mimic alias");
    require(action == ml::TextControlAction::Skill, "track_mimic should map to skill");
    require(ml::text_control_action_from_word("fail_safe", action), "fail_safe alias");
    require(action == ml::TextControlAction::FinalDamping, "fail_safe should map to final damping");
    require(ml::text_control_action_from_word("x", action), "x alias");
    require(action == ml::TextControlAction::Zero, "x should map to zero");
    require(!ml::text_control_action_from_word("teleport", action), "teleport should be invalid");
}

void check_order_preserved()
{
    const auto ops = ml::parse_text_control_operations("zero vx=0.4 pause resume stop");
    require(ops.size() == 5, "order preserved op count");
    expect_action(ops[0], ml::TextControlAction::Zero);
    expect_velocity(ops[1], 0, 0.4f);
    expect_action(ops[2], ml::TextControlAction::Pause);
    expect_action(ops[3], ml::TextControlAction::Resume);
    expect_action(ops[4], ml::TextControlAction::Stop);
}

void check_invalid_tokens_are_ignored()
{
    const auto ops = ml::parse_text_control_operations("vx=nan mode=teleport garbage vy=0.3");
    require(ops.size() == 1, "invalid tokens ignored op count");
    expect_velocity(ops[0], 1, 0.3f);
}

void check_action_effects()
{
    const auto loco = ml::text_control_action_effect(ml::TextControlAction::Loco);
    require(loco.mode_requested, "loco effect should request a mode");
    require(loco.mode == ml::ControlMode::Loco, "loco effect mode");
    require(!loco.zero_command, "loco effect should preserve command");
    require(loco.unpause, "loco effect should unpause");

    const auto passive = ml::text_control_action_effect(ml::TextControlAction::Passive);
    require(passive.mode_requested, "passive effect should request a mode");
    require(passive.mode == ml::ControlMode::Passive, "passive effect mode");
    require(passive.zero_command, "passive effect should zero command");
    require(passive.unpause, "passive effect should unpause");

    const auto dance = ml::text_control_action_effect(ml::TextControlAction::Dance);
    require(dance.mode_requested, "dance effect should request a mode");
    require(dance.mode == ml::ControlMode::Dance, "dance effect mode");
    require(dance.external_policy_key == ml::kBeyondMimicPolicyKey, "dance effect external key");

    const auto skill = ml::text_control_action_effect(ml::TextControlAction::Skill);
    require(skill.mode_requested, "skill effect should request a mode");
    require(skill.mode == ml::ControlMode::Skill, "skill effect mode");
    require(skill.zero_command, "skill effect should zero command");
    require(skill.external_policy_key == ml::kTrackMimicPolicyKey, "skill effect external key");

    const auto reset = ml::text_control_action_effect(ml::TextControlAction::ResetStand);
    require(!reset.mode_requested, "reset effect should leave mode handling to entrypoint");
    require(reset.zero_command, "reset effect should zero command");
    require(reset.reset_stand, "reset effect should request re-stand/reset");

    const auto pause = ml::text_control_action_effect(ml::TextControlAction::Pause);
    require(pause.pause, "pause effect should pause");
    require(!pause.zero_command, "pause effect should not rewrite stored command");

    const auto stop = ml::text_control_action_effect(ml::TextControlAction::Stop);
    require(stop.stop, "stop effect should request stop");
}

}  // namespace

int main()
{
    try {
        check_named_velocity_and_mode();
        check_numeric_tokens_and_clear_modes();
        check_aliases();
        check_order_preserved();
        check_invalid_tokens_are_ignored();
        check_action_effects();
    } catch (const std::exception& error) {
        std::cerr << "[text_control_command_check][FAIL] " << error.what() << "\n";
        return 1;
    }

    std::cout << "[text_control_command_check] PASS\n";
    return 0;
}
