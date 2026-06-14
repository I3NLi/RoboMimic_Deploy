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
    require(ml::text_control_action_from_word("walk", action), "walk alias");
    require(action == ml::TextControlAction::WalkForward, "walk should map to walk-forward preset");
    require(ml::text_control_action_from_word("run_forward", action), "run_forward alias");
    require(action == ml::TextControlAction::RunForward, "run_forward should map to run-forward preset");
    require(ml::text_control_action_from_word("sprint", action), "sprint alias");
    require(action == ml::TextControlAction::RunForward, "sprint should map to run-forward preset");
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
    const auto ops = ml::parse_text_control_operations("zero walk vx=0.4 run_forward pause resume stop");
    require(ops.size() == 7, "order preserved op count");
    expect_action(ops[0], ml::TextControlAction::Zero);
    expect_action(ops[1], ml::TextControlAction::WalkForward);
    expect_velocity(ops[2], 0, 0.4f);
    expect_action(ops[3], ml::TextControlAction::RunForward);
    expect_action(ops[4], ml::TextControlAction::Pause);
    expect_action(ops[5], ml::TextControlAction::Resume);
    expect_action(ops[6], ml::TextControlAction::Stop);
}

void check_invalid_tokens_are_ignored()
{
    const auto ops = ml::parse_text_control_operations("vx=nan mode=teleport garbage vy=0.3");
    require(ops.size() == 1, "invalid tokens ignored op count");
    expect_velocity(ops[0], 1, 0.3f);
}

void check_action_effects()
{
    auto require_request = [](
        const ml::TextControlActionEffect& effect,
        ml::ControlMode mode,
        const std::string& external_policy_key,
        const char* label) {
        const auto request = ml::mode_request_for_text_control_effect(effect);
        require(request.requested, std::string(label) + " should build a mode request");
        require(request.mode == mode, std::string(label) + " request mode");
        require(request.external_policy_key == external_policy_key, std::string(label) + " request external key");
    };

    const auto loco = ml::text_control_action_effect(ml::TextControlAction::Loco);
    require(loco.mode_requested, "loco effect should request a mode");
    require(loco.mode == ml::ControlMode::Loco, "loco effect mode");
    require(!loco.zero_command, "loco effect should preserve command");
    require(!loco.command_requested, "loco effect should not set a preset command");
    require(loco.unpause, "loco effect should unpause");
    require_request(loco, ml::ControlMode::Loco, {}, "loco effect");

    const auto walk = ml::text_control_action_effect(ml::TextControlAction::WalkForward);
    require(walk.mode_requested, "walk effect should request a mode");
    require(walk.mode == ml::ControlMode::Loco, "walk effect mode");
    require(walk.command_requested, "walk effect should set a preset command");
    require(same_float(walk.command[0], ml::kTextControlWalkVx), "walk effect vx");
    require(same_float(walk.command[1], 0.0f), "walk effect vy");
    require(same_float(walk.command[2], 0.0f), "walk effect wz");
    require(walk.unpause, "walk effect should unpause");
    require_request(walk, ml::ControlMode::Loco, {}, "walk effect");

    const auto run = ml::text_control_action_effect(ml::TextControlAction::RunForward);
    require(run.mode_requested, "run-forward effect should request a mode");
    require(run.mode == ml::ControlMode::Loco, "run-forward effect mode");
    require(run.command_requested, "run-forward effect should set a preset command");
    require(same_float(run.command[0], ml::kTextControlRunVx), "run-forward effect vx");
    require(same_float(run.command[1], 0.0f), "run-forward effect vy");
    require(same_float(run.command[2], 0.0f), "run-forward effect wz");
    require(run.unpause, "run-forward effect should unpause");
    require_request(run, ml::ControlMode::Loco, {}, "run-forward effect");

    const auto stand = ml::text_control_action_effect(ml::TextControlAction::Stand);
    require(stand.mode_requested, "stand effect should request a mode");
    require(stand.mode == ml::ControlMode::Stand, "stand effect mode");
    require(stand.zero_command, "stand effect should zero command");
    require_request(stand, ml::ControlMode::Stand, {}, "stand effect");

    const auto passive = ml::text_control_action_effect(ml::TextControlAction::Passive);
    require(passive.mode_requested, "passive effect should request a mode");
    require(passive.mode == ml::ControlMode::Passive, "passive effect mode");
    require(passive.zero_command, "passive effect should zero command");
    require(passive.unpause, "passive effect should unpause");
    require_request(passive, ml::ControlMode::Passive, {}, "passive effect");

    const auto dance = ml::text_control_action_effect(ml::TextControlAction::Dance);
    require(dance.mode_requested, "dance effect should request a mode");
    require(dance.mode == ml::ControlMode::Dance, "dance effect mode");
    require(dance.external_policy_key == ml::kBeyondMimicPolicyKey, "dance effect external key");
    require_request(dance, ml::ControlMode::Dance, ml::kBeyondMimicPolicyKey, "dance effect");

    const auto skill = ml::text_control_action_effect(ml::TextControlAction::Skill);
    require(skill.mode_requested, "skill effect should request a mode");
    require(skill.mode == ml::ControlMode::Skill, "skill effect mode");
    require(skill.zero_command, "skill effect should zero command");
    require(skill.external_policy_key == ml::kTrackMimicPolicyKey, "skill effect external key");
    require_request(skill, ml::ControlMode::Skill, ml::kTrackMimicPolicyKey, "skill effect");

    const auto final_damping = ml::text_control_action_effect(ml::TextControlAction::FinalDamping);
    require(final_damping.mode_requested, "final damping effect should request a mode");
    require(final_damping.mode == ml::ControlMode::FinalDamping, "final damping effect mode");
    require(final_damping.zero_command, "final damping effect should zero command");
    require_request(final_damping, ml::ControlMode::FinalDamping, {}, "final damping effect");

    const auto zero = ml::text_control_action_effect(ml::TextControlAction::Zero);
    const auto zero_request = ml::mode_request_for_text_control_effect(zero);
    require(!zero_request.requested, "non-mode effect should not build a mode request");

    const auto reset = ml::text_control_action_effect(ml::TextControlAction::ResetStand);
    require(reset.mode_requested, "reset effect should request stand mode");
    require(reset.mode == ml::ControlMode::Stand, "reset effect mode");
    require(reset.zero_command, "reset effect should zero command");
    require(reset.reset_stand, "reset effect should request re-stand/reset");
    require_request(reset, ml::ControlMode::Stand, {}, "reset effect");

    const auto pause = ml::text_control_action_effect(ml::TextControlAction::Pause);
    require(pause.pause, "pause effect should pause");
    require(!pause.zero_command, "pause effect should not rewrite stored command");

    const auto stop = ml::text_control_action_effect(ml::TextControlAction::Stop);
    require(stop.stop, "stop effect should request stop");
}

void check_intent_application()
{
    ml::TextControlIntentState intent;
    intent.command = {0.2f, -0.1f, 0.05f};

    auto result = ml::apply_text_control_action_to_intent(intent, ml::TextControlAction::Loco);
    require(result.applied, "loco intent should apply");
    require(intent.desired_mode == ml::ControlMode::Loco, "loco intent mode");
    require(intent.reset_requested, "entering loco should request reset");
    require(same_float(intent.command[0], 0.2f), "loco intent should preserve vx");

    result = ml::apply_text_control_action_to_intent(intent, ml::TextControlAction::Pause);
    require(result.applied, "pause intent should apply");
    require(intent.paused, "pause intent should pause");
    require(same_float(intent.command[0], 0.2f), "pause intent should preserve stored command");

    result = ml::apply_text_control_action_to_intent(intent, ml::TextControlAction::Resume);
    require(result.applied, "resume intent should apply");
    require(!intent.paused, "resume intent should unpause");

    intent.reset_requested = false;
    intent.desired_mode = ml::ControlMode::Stand;
    intent.command = {0.0f, 0.0f, 0.0f};
    result = ml::apply_text_control_action_to_intent(intent, ml::TextControlAction::WalkForward);
    require(result.applied, "walk intent should apply");
    require(intent.desired_mode == ml::ControlMode::Loco, "walk intent mode");
    require(intent.reset_requested, "walk intent should request reset entering loco");
    require(same_float(intent.command[0], ml::kTextControlWalkVx), "walk intent vx");
    require(same_float(intent.command[1], 0.0f), "walk intent vy");
    require(same_float(intent.command[2], 0.0f), "walk intent wz");

    intent.reset_requested = false;
    result = ml::apply_text_control_action_to_intent(intent, ml::TextControlAction::RunForward);
    require(result.applied, "run-forward intent should apply");
    require(intent.desired_mode == ml::ControlMode::Loco, "run-forward intent mode");
    require(!intent.reset_requested, "run-forward intent should not reset while already loco");
    require(same_float(intent.command[0], ml::kTextControlRunVx), "run-forward intent vx");
    require(same_float(intent.command[1], 0.0f), "run-forward intent vy");
    require(same_float(intent.command[2], 0.0f), "run-forward intent wz");

    intent.command = {0.6f, -0.2f, 0.4f};
    result = ml::apply_text_control_action_to_intent(intent, ml::TextControlAction::Zero);
    require(result.applied, "zero intent should apply");
    require(intent.command == std::array<float, 3>{0.0f, 0.0f, 0.0f}, "zero intent should zero command");
    require(intent.desired_mode == ml::ControlMode::Loco, "zero intent should not change mode");
    require(!intent.paused, "zero intent should not pause");

    intent.reset_requested = false;
    result = ml::apply_text_control_action_to_intent(intent, ml::TextControlAction::Stand);
    require(result.applied, "stand intent should apply");
    require(intent.desired_mode == ml::ControlMode::Stand, "stand intent mode");
    require(intent.reset_requested, "stand intent should request reset");
    require(intent.command == std::array<float, 3>{0.0f, 0.0f, 0.0f}, "stand intent should zero command");

    intent.reset_requested = false;
    intent.command = {0.3f, 0.0f, 0.0f};
    result = ml::apply_text_control_action_to_intent(intent, ml::TextControlAction::ToggleLoco);
    require(result.applied, "toggle-to-loco intent should apply");
    require(intent.desired_mode == ml::ControlMode::Loco, "toggle-to-loco intent mode");
    require(intent.reset_requested, "toggle-to-loco should request reset");
    require(same_float(intent.command[0], 0.3f), "toggle-to-loco should preserve command");

    intent.reset_requested = false;
    result = ml::apply_text_control_action_to_intent(intent, ml::TextControlAction::ToggleLoco);
    require(result.applied, "toggle-to-stand intent should apply");
    require(intent.desired_mode == ml::ControlMode::Stand, "toggle-to-stand intent mode");
    require(!intent.reset_requested, "toggle-to-stand should not force reset");
    require(intent.command == std::array<float, 3>{0.0f, 0.0f, 0.0f}, "toggle-to-stand should zero command");

    intent.command = {0.1f, 0.2f, 0.3f};
    result = ml::apply_text_control_action_to_intent(intent, ml::TextControlAction::ResetStand);
    require(result.applied, "reset intent should apply");
    require(intent.desired_mode == ml::ControlMode::Stand, "reset intent mode");
    require(intent.reset_requested, "reset intent should set reset flag");
    require(intent.command == std::array<float, 3>{0.0f, 0.0f, 0.0f}, "reset intent should zero command");

    intent.reset_requested = false;
    intent.paused = true;
    intent.command = {0.1f, 0.2f, 0.3f};
    result = ml::apply_text_control_action_to_intent(intent, ml::TextControlAction::FinalDamping);
    require(result.applied, "final damping intent should apply");
    require(intent.desired_mode == ml::ControlMode::FinalDamping, "final damping intent mode");
    require(intent.desired_external_policy_key.empty(), "final damping intent external key");
    require(!intent.reset_requested, "final damping intent should not request reset");
    require(!intent.paused, "final damping intent should unpause");
    require(intent.command == std::array<float, 3>{0.0f, 0.0f, 0.0f}, "final damping intent should zero command");

    intent.command = {0.1f, 0.2f, 0.3f};
    result = ml::apply_text_control_action_to_intent(intent, ml::TextControlAction::Dance);
    require(result.applied, "dance intent should apply");
    require(intent.desired_mode == ml::ControlMode::Dance, "dance intent mode");
    require(intent.desired_external_policy_key == ml::kBeyondMimicPolicyKey, "dance intent external key");
    require(intent.command == std::array<float, 3>{0.0f, 0.0f, 0.0f}, "dance intent should zero command");

    intent.command = {0.1f, 0.2f, 0.3f};
    result = ml::apply_text_control_action_to_intent(intent, ml::TextControlAction::Skill);
    require(result.applied, "skill intent should apply");
    require(intent.desired_mode == ml::ControlMode::Skill, "skill intent mode");
    require(intent.desired_external_policy_key == ml::kTrackMimicPolicyKey, "skill intent external key");
    require(intent.command == std::array<float, 3>{0.0f, 0.0f, 0.0f}, "skill intent should zero command");

    result = ml::apply_text_control_action_to_intent(intent, ml::TextControlAction::Stop);
    require(result.applied, "stop intent should apply");
    require(!intent.running, "stop intent should clear running");

    const auto before_disabled = intent;
    result = ml::apply_text_control_action_to_intent(
        intent,
        ml::TextControlAction::Dance,
        ml::TextControlIntentOptions{false, true});
    require(!result.applied, "disabled dance should be rejected");
    require(
        result.reject_reason == ml::TextControlIntentRejectReason::DanceDisabled,
        "disabled dance reject reason");
    require(intent.command == before_disabled.command, "disabled dance should not change command");
    require(intent.desired_mode == before_disabled.desired_mode, "disabled dance should not change mode");

    result = ml::apply_text_control_action_to_intent(
        intent,
        ml::TextControlAction::Skill,
        ml::TextControlIntentOptions{true, false});
    require(!result.applied, "disabled skill should be rejected");
    require(
        result.reject_reason == ml::TextControlIntentRejectReason::SkillDisabled,
        "disabled skill reject reason");
    require(intent.command == before_disabled.command, "disabled skill should not change command");
    require(intent.desired_mode == before_disabled.desired_mode, "disabled skill should not change mode");
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
        check_intent_application();
    } catch (const std::exception& error) {
        std::cerr << "[text_control_command_check][FAIL] " << error.what() << "\n";
        return 1;
    }

    std::cout << "[text_control_command_check] PASS\n";
    return 0;
}
