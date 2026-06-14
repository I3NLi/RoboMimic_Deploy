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

}  // namespace

int main()
{
    try {
        check_named_velocity_and_mode();
        check_numeric_tokens_and_clear_modes();
        check_aliases();
        check_order_preserved();
        check_invalid_tokens_are_ignored();
    } catch (const std::exception& error) {
        std::cerr << "[text_control_command_check][FAIL] " << error.what() << "\n";
        return 1;
    }

    std::cout << "[text_control_command_check] PASS\n";
    return 0;
}
