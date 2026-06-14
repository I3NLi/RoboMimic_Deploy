#include "mode_manager.h"

#include <iostream>
#include <stdexcept>
#include <string>

namespace ml = magicbot_loco;

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void expect_transition(
    const ml::ModeTransition& transition,
    ml::ControlMode previous,
    ml::ControlMode current,
    bool changed,
    bool reset_policy_history,
    bool zero_command,
    bool seed_target_from_state,
    const std::string& label)
{
    require(transition.previous == previous, label + ": previous mode");
    require(transition.current == current, label + ": current mode");
    require(transition.changed == changed, label + ": changed");
    require(transition.reset_policy_history == reset_policy_history, label + ": reset policy history");
    require(transition.zero_command == zero_command, label + ": zero command");
    require(transition.seed_target_from_state == seed_target_from_state, label + ": seed target");
}

void check_default_modes()
{
    ml::ModeManager manager;
    require(manager.mode() == ml::ControlMode::Stand, "default mode should be STAND");
    require(manager.is_enabled(ml::ControlMode::Passive), "PASSIVE enabled by default");
    require(manager.is_enabled(ml::ControlMode::Stand), "STAND enabled by default");
    require(manager.is_enabled(ml::ControlMode::Loco), "LOCO enabled by default");
    require(manager.is_enabled(ml::ControlMode::FinalDamping), "FINAL_DAMPING enabled by default");
    require(!manager.is_enabled(ml::ControlMode::Dance), "DANCE disabled until policy registration");
    require(!manager.is_enabled(ml::ControlMode::Skill), "SKILL disabled until policy registration");
}

void check_core_transitions()
{
    ml::ModeManager manager;

    expect_transition(
        manager.apply(ml::ModeRequest::none()),
        ml::ControlMode::Stand,
        ml::ControlMode::Stand,
        false,
        false,
        false,
        false,
        "none request");

    expect_transition(
        manager.apply(ml::ModeRequest::enter(ml::ControlMode::Loco)),
        ml::ControlMode::Stand,
        ml::ControlMode::Loco,
        true,
        true,
        false,
        false,
        "stand to loco");

    expect_transition(
        manager.apply(ml::ModeRequest::enter(ml::ControlMode::Passive)),
        ml::ControlMode::Loco,
        ml::ControlMode::Passive,
        true,
        true,
        true,
        true,
        "loco to passive");

    expect_transition(
        manager.apply(ml::ModeRequest::enter(ml::ControlMode::Stand)),
        ml::ControlMode::Passive,
        ml::ControlMode::Stand,
        true,
        false,
        true,
        false,
        "passive to stand");

    expect_transition(
        manager.apply(ml::ModeRequest::enter(ml::ControlMode::FinalDamping)),
        ml::ControlMode::Stand,
        ml::ControlMode::FinalDamping,
        true,
        false,
        true,
        true,
        "stand to final damping");

    expect_transition(
        manager.apply(ml::ModeRequest::enter(ml::ControlMode::FinalDamping)),
        ml::ControlMode::FinalDamping,
        ml::ControlMode::FinalDamping,
        false,
        false,
        false,
        false,
        "same final damping");
}

void check_external_policy_modes()
{
    ml::ModeManager manager;
    bool threw = false;
    try {
        (void)manager.apply(ml::ModeRequest::enter_external(
            ml::ControlMode::Dance,
            ml::kBeyondMimicPolicyKey));
    } catch (const std::runtime_error&) {
        threw = true;
    }
    require(threw, "DANCE request should throw until enabled");

    manager.set_enabled(ml::ControlMode::Dance, true);
    expect_transition(
        manager.apply(ml::ModeRequest::enter_external(
            ml::ControlMode::Dance,
            ml::kBeyondMimicPolicyKey)),
        ml::ControlMode::Stand,
        ml::ControlMode::Dance,
        true,
        true,
        true,
        false,
        "stand to dance");

    expect_transition(
        manager.apply(ml::ModeRequest::enter(ml::ControlMode::Loco)),
        ml::ControlMode::Dance,
        ml::ControlMode::Loco,
        true,
        true,
        false,
        false,
        "dance to loco");
}

void check_mode_request_helpers()
{
    const ml::ModeRequest stand = ml::mode_request_for_control_mode(ml::ControlMode::Stand);
    require(stand.requested, "stand helper should request a mode");
    require(stand.mode == ml::ControlMode::Stand, "stand helper mode");
    require(stand.external_policy_key.empty(), "stand helper should not set external key");

    const ml::ModeRequest dance = ml::mode_request_for_control_mode(ml::ControlMode::Dance);
    require(dance.requested, "dance helper should request a mode");
    require(dance.mode == ml::ControlMode::Dance, "dance helper mode");
    require(dance.external_policy_key == ml::kBeyondMimicPolicyKey, "dance helper should select BeyondMimic");

    const ml::ModeRequest skill =
        ml::mode_request_for_control_mode(ml::ControlMode::Skill, "TrackMimic");
    require(skill.requested, "skill helper should request a mode");
    require(skill.mode == ml::ControlMode::Skill, "skill helper mode");
    require(skill.external_policy_key == "TrackMimic", "skill helper should preserve explicit key");
}

}  // namespace

int main()
{
    try {
        check_default_modes();
        check_core_transitions();
        check_external_policy_modes();
        check_mode_request_helpers();
    } catch (const std::exception& error) {
        std::cerr << "[mode_manager_check][FAIL] " << error.what() << "\n";
        return 1;
    }

    std::cout << "[mode_manager_check] PASS\n";
    return 0;
}
