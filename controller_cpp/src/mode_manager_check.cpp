#include "mode_manager.h"
#include "native_fsm_states.h"

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

    const ml::ModeRequest default_skill = ml::mode_request_for_control_mode(ml::ControlMode::Skill);
    require(default_skill.requested, "default skill helper should request a mode");
    require(default_skill.mode == ml::ControlMode::Skill, "default skill helper mode");
    require(
        default_skill.external_policy_key == ml::kTrackMimicPolicyKey,
        "default skill helper should select TrackMimic trajectory key");

    const ml::ModeRequest steady_stand = ml::mode_request_for_desired_control_mode(
        ml::ControlMode::Stand,
        {},
        ml::ControlMode::Stand);
    require(!steady_stand.requested, "desired helper should not re-request current stand");

    const ml::ModeRequest desired_loco = ml::mode_request_for_desired_control_mode(
        ml::ControlMode::Loco,
        {},
        ml::ControlMode::Stand);
    require(desired_loco.requested, "desired helper should request a different mode");
    require(desired_loco.mode == ml::ControlMode::Loco, "desired helper different mode");

    const ml::ModeRequest toggle_from_stand = ml::mode_request_for_loco_toggle(ml::ControlMode::Stand);
    require(toggle_from_stand.requested, "toggle helper should request from stand");
    require(toggle_from_stand.mode == ml::ControlMode::Loco, "toggle helper should enter loco from stand");
    require(toggle_from_stand.external_policy_key.empty(), "toggle helper should clear external key entering loco");

    const ml::ModeRequest toggle_from_loco = ml::mode_request_for_loco_toggle(ml::ControlMode::Loco);
    require(toggle_from_loco.requested, "toggle helper should request from loco");
    require(toggle_from_loco.mode == ml::ControlMode::Stand, "toggle helper should return to stand from loco");
    require(toggle_from_loco.external_policy_key.empty(), "toggle helper should clear external key leaving loco");

    const ml::ModeRequest toggle_from_skill = ml::mode_request_for_loco_toggle(ml::ControlMode::Skill);
    require(toggle_from_skill.requested, "toggle helper should request from external mode");
    require(toggle_from_skill.mode == ml::ControlMode::Loco, "toggle helper should enter loco from external mode");

    const ml::ModeRequest steady_skill = ml::mode_request_for_desired_control_mode(
        ml::ControlMode::Skill,
        ml::kTrackMimicPolicyKey,
        ml::ControlMode::Skill,
        ml::kTrackMimicPolicyKey);
    require(!steady_skill.requested, "desired helper should not re-request same external policy");

    const ml::ModeRequest changed_skill_key = ml::mode_request_for_desired_control_mode(
        ml::ControlMode::Skill,
        "TrackMimicVariant",
        ml::ControlMode::Skill,
        ml::kTrackMimicPolicyKey);
    require(changed_skill_key.requested, "desired helper should request changed external policy key");
    require(changed_skill_key.mode == ml::ControlMode::Skill, "desired helper changed external mode");
    require(
        changed_skill_key.external_policy_key == "TrackMimicVariant",
        "desired helper should preserve changed external policy key");
}

void check_native_fsm_mode_mapper()
{
    require(
        control_mode_for_fsm_state(FSMStateName::PASSIVE) == ml::ControlMode::Passive,
        "native PASSIVE should map to PASSIVE");
    require(
        control_mode_for_fsm_state(FSMStateName::FIXEDPOSE) == ml::ControlMode::Stand,
        "native FIXEDPOSE should map to STAND");
    require(
        control_mode_for_fsm_state(FSMStateName::LOCOMODE) == ml::ControlMode::Loco,
        "native LOCOMODE should map to LOCO");
    require(
        control_mode_for_fsm_state(FSMStateName::SKILL_COOLDOWN) == ml::ControlMode::Loco,
        "native SKILL_COOLDOWN should map back to LOCO");
    require(
        control_mode_for_fsm_state(FSMStateName::SKILL_BEYOND_MIMIC) == ml::ControlMode::Dance,
        "native BeyondMimic should map to DANCE");
    require(
        control_mode_for_fsm_state(FSMStateName::SKILL_TRACK_MIMIC) == ml::ControlMode::Skill,
        "native BeyondMimic trajectory/TrackMimic should map to SKILL");
}

}  // namespace

int main()
{
    try {
        check_default_modes();
        check_core_transitions();
        check_external_policy_modes();
        check_mode_request_helpers();
        check_native_fsm_mode_mapper();
    } catch (const std::exception& error) {
        std::cerr << "[mode_manager_check][FAIL] " << error.what() << "\n";
        return 1;
    }

    std::cout << "[mode_manager_check] PASS\n";
    return 0;
}
