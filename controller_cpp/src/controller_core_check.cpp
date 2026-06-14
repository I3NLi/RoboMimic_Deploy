#include "controller_runtime.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace ml = magicbot_loco;

namespace {

ml::JointArray offset_target(ml::JointArray target, float scale);

class FakeAdapter : public ml::RobotAdapter {
public:
    explicit FakeAdapter(ml::RobotSnapshot snapshot)
        : snapshot_(snapshot)
    {
    }

    const char* name() const override { return "fake"; }

    ml::RobotSnapshot read_snapshot() override
    {
        ++reads;
        return snapshot_;
    }

    void write_target(const ml::JointTarget& target) override
    {
        ++writes;
        last_target = target;
    }

    void write_damping(float damping_kd) override
    {
        ++damping_writes;
        last_damping_kd = damping_kd;
    }

    ml::AdapterTelemetry telemetry() const override
    {
        ml::AdapterTelemetry out;
        out.backend = name();
        out.command_published = writes > 0;
        return out;
    }

    int reads{0};
    int writes{0};
    int damping_writes{0};
    float last_damping_kd{0.0f};
    ml::JointTarget last_target{};

private:
    ml::RobotSnapshot snapshot_;
};

class FakeExternalPolicy : public ml::ExternalPolicyAdapter {
public:
    FakeExternalPolicy(ml::ControlMode mode, const char* name, float target_scale = 0.002f)
        : mode_(mode),
          name_(name),
          target_scale_(target_scale)
    {
    }

    ml::ControlMode mode() const override { return mode_; }
    const char* name() const override { return name_; }

    void reset(const ml::RobotSnapshot& snapshot) override
    {
        ++resets;
        last_reset_q = snapshot.q;
    }

    ml::ExternalPolicyOutput step(const ml::ExternalPolicyInput& input) override
    {
        ++steps;
        last_velocity = input.velocity_command;
        ml::ExternalPolicyOutput out;
        out.target_motor = offset_target(input.snapshot.q, target_scale_);
        out.complete = complete_next_step;
        out.next_mode = next_mode;
        complete_next_step = false;
        return out;
    }

    int resets{0};
    int steps{0};
    bool complete_next_step{false};
    ml::ControlMode next_mode{ml::ControlMode::Stand};
    ml::JointArray last_reset_q{};
    std::array<float, 3> last_velocity{0.0f, 0.0f, 0.0f};

private:
    ml::ControlMode mode_;
    const char* name_;
    float target_scale_{0.002f};
};

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool near(float lhs, float rhs)
{
    return std::fabs(lhs - rhs) < 1e-5f;
}

void require_joint_array_near(
    const ml::JointArray& lhs,
    const ml::JointArray& rhs,
    const std::string& label)
{
    for (int i = 0; i < ml::kNumJoints; ++i) {
        if (!near(lhs[static_cast<size_t>(i)], rhs[static_cast<size_t>(i)])) {
            throw std::runtime_error(label + ": joint " + std::to_string(i));
        }
    }
}

void require_vec3_near(
    const std::array<float, 3>& lhs,
    const std::array<float, 3>& rhs,
    const std::string& label)
{
    for (int i = 0; i < 3; ++i) {
        if (!near(lhs[static_cast<size_t>(i)], rhs[static_cast<size_t>(i)])) {
            throw std::runtime_error(label + ": axis " + std::to_string(i));
        }
    }
}

ml::RobotSnapshot make_snapshot(const ml::JointArray& q)
{
    ml::RobotSnapshot snapshot;
    snapshot.q = q;
    snapshot.quat = {1.0f, 0.0f, 0.0f, 0.0f};
    snapshot.counts = {12, 8, 1, 1};
    return snapshot;
}

ml::JointArray offset_target(ml::JointArray target, float scale)
{
    for (int i = 0; i < ml::kNumJoints; ++i) {
        target[static_cast<size_t>(i)] += scale * static_cast<float>((i % 5) - 2);
    }
    return target;
}

void check_stand_passive_final_modes(const std::filesystem::path& config_path)
{
    ml::LocoConfig cfg = ml::load_loco_config(config_path);
    ml::ControllerCoreOptions options;
    options.safety.enabled = false;
    ml::ControllerCore core(cfg, options);

    const ml::JointArray stand_q = cfg.default_motor();
    const ml::JointArray shifted_q = offset_target(stand_q, 0.01f);
    const ml::RobotSnapshot stand_snapshot = make_snapshot(stand_q);
    const ml::RobotSnapshot shifted_snapshot = make_snapshot(shifted_q);

    const auto stand = core.step(
        stand_snapshot,
        ml::Command{},
        ml::mode_request_for_control_mode(ml::ControlMode::Stand),
        static_cast<float>(ml::kControlDt));
    require(stand.telemetry.mode == ml::ControlMode::Stand, "stand mode telemetry");
    require(!stand.target.damping_only, "stand should publish position target");
    require(!stand.telemetry.policy_evaluated, "stand should not evaluate policy");

    const auto passive = core.step(
        shifted_snapshot,
        ml::Command{{0.4f, -0.2f, 0.1f}},
        ml::mode_request_for_control_mode(ml::ControlMode::Passive),
        static_cast<float>(ml::kControlDt));
    require(passive.telemetry.mode == ml::ControlMode::Passive, "passive mode telemetry");
    require(passive.target.damping_only, "passive should be damping-only");
    require(!passive.telemetry.policy_evaluated, "passive should not evaluate policy");
    require_joint_array_near(passive.target.q, shifted_q, "passive target should seed from state");

    const ml::JointArray final_q = offset_target(stand_q, -0.012f);
    const auto final_damping = core.step(
        make_snapshot(final_q),
        ml::Command{{-0.5f, 0.1f, 0.3f}},
        ml::mode_request_for_control_mode(ml::ControlMode::FinalDamping),
        static_cast<float>(ml::kControlDt));
    require(final_damping.telemetry.mode == ml::ControlMode::FinalDamping, "final damping mode telemetry");
    require(final_damping.target.damping_only, "final damping should be damping-only");
    require(!final_damping.telemetry.policy_evaluated, "final damping should not evaluate policy");
    require_joint_array_near(final_damping.target.q, final_q, "final damping target should seed from state");
}

void check_runtime_adapter_flow(const std::filesystem::path& config_path)
{
    ml::LocoConfig cfg = ml::load_loco_config(config_path);
    ml::ControllerCoreOptions options;
    options.safety.enabled = false;
    ml::ControllerCore core(cfg, options);

    const ml::JointArray q = offset_target(cfg.default_motor(), 0.006f);
    FakeAdapter adapter(make_snapshot(q));
    ml::ControllerRuntime runtime(core, adapter);

    ml::RuntimeTickInput tick_input;
    tick_input.mode_request = ml::mode_request_for_control_mode(ml::ControlMode::Passive);
    tick_input.control_dt_s = static_cast<float>(ml::kControlDt);
    const auto tick = runtime.tick(tick_input);
    require(adapter.reads == 1, "runtime should read adapter once");
    require(adapter.writes == 1, "runtime should write target when publish_target=true");
    require(tick.adapter.backend == "fake", "runtime should return adapter telemetry");
    require(tick.adapter.command_published, "adapter telemetry should report command published");
    require(tick.core.telemetry.mode == ml::ControlMode::Passive, "runtime core mode telemetry");
    require(adapter.last_target.damping_only, "runtime should publish damping-only passive target");
    require_joint_array_near(adapter.last_target.q, q, "runtime published target should come from snapshot");

    tick_input.mode_request = ml::ModeRequest::none();
    tick_input.publish_target = false;
    (void)runtime.tick(tick_input);
    require(adapter.reads == 2, "runtime should still read when publish_target=false");
    require(adapter.writes == 1, "runtime should not write when publish_target=false");

    ml::JointTarget held_target = adapter.last_target;
    held_target.q = offset_target(held_target.q, 0.003f);
    runtime.write_target(held_target);
    require(adapter.writes == 2, "runtime write_target should call adapter");
    require_joint_array_near(adapter.last_target.q, held_target.q, "runtime write_target target");

    runtime.write_damping(4.5f);
    require(adapter.damping_writes == 1, "runtime write_damping should call adapter");
    require(near(adapter.last_damping_kd, 4.5f), "runtime write_damping kd");
}

void check_shared_target_rate_limit(const std::filesystem::path& config_path)
{
    ml::LocoConfig cfg = ml::load_loco_config(config_path);
    ml::ControllerCoreOptions options;
    options.safety.enabled = false;
    options.max_target_rate = 0.02f;
    ml::ControllerCore core(cfg, options);

    const ml::JointArray start_q = cfg.default_motor();
    ml::JointArray desired_q = start_q;
    for (float& q : desired_q) q += 0.08f;

    core.set_default_target(desired_q);
    core.seed_target(start_q);

    const float dt = 0.5f;
    const float max_delta = options.max_target_rate * dt;
    const auto out = core.step(
        make_snapshot(start_q),
        ml::Command{},
        ml::mode_request_for_control_mode(ml::ControlMode::Stand),
        dt);

    require(out.telemetry.mode == ml::ControlMode::Stand, "rate-limit stand mode telemetry");
    for (int i = 0; i < ml::kNumJoints; ++i) {
        const float expected = start_q[static_cast<size_t>(i)] + max_delta;
        if (!near(out.target.q[static_cast<size_t>(i)], expected)) {
            throw std::runtime_error("shared target rate limit: joint " + std::to_string(i));
        }
        if (!near(out.telemetry.command_target[static_cast<size_t>(i)], expected)) {
            throw std::runtime_error("shared target telemetry rate limit: joint " + std::to_string(i));
        }
    }
}

void check_shared_motion_safety(const std::filesystem::path& config_path)
{
    ml::LocoConfig cfg = ml::load_loco_config(config_path);
    ml::ControllerCoreOptions options;
    options.safety.enabled = true;
    options.safety.max_gravity_xy = 0.01f;
    ml::ControllerCore core(cfg, options);

    ml::RobotSnapshot unsafe = make_snapshot(cfg.default_motor());
    unsafe.quat = {0.7071068f, 0.0f, 0.7071068f, 0.0f};

    bool threw = false;
    try {
        (void)core.step(
            unsafe,
            ml::Command{},
            ml::mode_request_for_control_mode(ml::ControlMode::Stand),
            static_cast<float>(ml::kControlDt));
    } catch (const std::runtime_error& error) {
        threw = std::string(error.what()).find("motion safety: projected gravity xy") != std::string::npos;
    }
    require(threw, "ControllerCore should run shared motion safety checks");
}

void check_external_policy_flow(const std::filesystem::path& config_path)
{
    ml::LocoConfig cfg = ml::load_loco_config(config_path);
    ml::ControllerCoreOptions options;
    options.safety.enabled = false;

    {
        ml::ControllerCore core(cfg, options);
        bool threw = false;
        try {
            (void)core.step(
                make_snapshot(cfg.default_motor()),
                ml::Command{},
                ml::mode_request_for_control_mode(ml::ControlMode::Dance, "MissingDance"),
                cfg.policy_dt);
        } catch (const std::runtime_error&) {
            threw = true;
        }
        require(threw, "unregistered DANCE policy should reject mode request");
    }

    {
        ml::ControllerCore core(cfg, options);
        FakeExternalPolicy dance(ml::ControlMode::Dance, "FakeDance");
        core.register_external_policy("FakeDance", dance, true);

        const ml::JointArray q = offset_target(cfg.default_motor(), 0.004f);
        const ml::RobotSnapshot snapshot = make_snapshot(q);
        const ml::Command command{{0.4f, -0.2f, 0.15f}};
        const auto entered = core.step(
            snapshot,
            command,
            ml::mode_request_for_control_mode(ml::ControlMode::Dance, "FakeDance"),
            cfg.policy_dt);
        require(entered.telemetry.mode == ml::ControlMode::Dance, "dance mode telemetry");
        require(entered.telemetry.external_policy == "FakeDance", "dance external policy telemetry");
        require(entered.telemetry.policy_evaluated, "dance should evaluate external policy");
        require(dance.resets == 1, "dance policy should reset on entry");
        require(dance.steps == 1, "dance policy should step on entry");
        require_joint_array_near(dance.last_reset_q, q, "dance reset should receive snapshot");
        require_vec3_near(dance.last_velocity, {0.0f, 0.0f, 0.0f}, "dance entry should zero command");

        const auto continued = core.step(
            snapshot,
            command,
            ml::ModeRequest::none(),
            cfg.policy_dt);
        require(continued.telemetry.mode == ml::ControlMode::Dance, "continued dance mode telemetry");
        require(dance.steps == 2, "dance policy should step again after policy dt");
        require_vec3_near(dance.last_velocity, command.velocity, "continued dance should preserve command");

        dance.complete_next_step = true;
        dance.next_mode = ml::ControlMode::FinalDamping;
        const auto completed = core.step(
            snapshot,
            command,
            ml::ModeRequest::none(),
            cfg.policy_dt);
        require(completed.telemetry.mode == ml::ControlMode::FinalDamping, "completed dance next mode");
        require(completed.target.damping_only, "completed dance should enter damping target");
        require_joint_array_near(completed.target.q, q, "completed dance should seed target from state");
    }

    {
        ml::ControllerCore core(cfg, options);
        FakeExternalPolicy skill(ml::ControlMode::Skill, "FakeSkill");
        core.register_external_policy("FakeSkill", skill, true);

        const auto out = core.step(
            make_snapshot(offset_target(cfg.default_motor(), -0.004f)),
            ml::Command{{-0.1f, 0.2f, -0.3f}},
            ml::mode_request_for_control_mode(ml::ControlMode::Skill, "FakeSkill"),
            cfg.policy_dt);
        require(out.telemetry.mode == ml::ControlMode::Skill, "skill mode telemetry");
        require(out.telemetry.external_policy == "FakeSkill", "skill external policy telemetry");
        require(out.telemetry.policy_evaluated, "skill should evaluate external policy");
        require(skill.resets == 1, "skill policy should reset on entry");
        require(skill.steps == 1, "skill policy should step on entry");
    }

    {
        ml::ControllerCore core(cfg, options);
        FakeExternalPolicy first(ml::ControlMode::Skill, "SkillA");
        FakeExternalPolicy second(ml::ControlMode::Skill, "SkillB");
        core.register_external_policy("SkillA", first, true);
        core.register_external_policy("SkillB", second, false);

        const ml::RobotSnapshot snapshot = make_snapshot(offset_target(cfg.default_motor(), 0.006f));
        const ml::Command command{{0.2f, 0.1f, -0.2f}};
        const auto first_out = core.step(
            snapshot,
            command,
            ml::mode_request_for_control_mode(ml::ControlMode::Skill, "SkillA"),
            cfg.policy_dt);
        require(first_out.telemetry.external_policy == "SkillA", "first skill key telemetry");
        require(first.resets == 1, "first skill policy should reset on entry");
        require(first.steps == 1, "first skill policy should step on entry");

        const auto second_out = core.step(
            snapshot,
            command,
            ml::mode_request_for_control_mode(ml::ControlMode::Skill, "SkillB"),
            cfg.policy_dt);
        require(second_out.telemetry.mode == ml::ControlMode::Skill, "same-mode skill switch mode");
        require(second_out.telemetry.external_policy == "SkillB", "same-mode skill switch telemetry");
        require(first.steps == 1, "first skill policy should stop stepping after switch");
        require(second.resets == 1, "second skill policy should reset on same-mode key switch");
        require(second.steps == 1, "second skill policy should step after same-mode key switch");
        require_vec3_near(
            second.last_velocity,
            {0.0f, 0.0f, 0.0f},
            "same-mode external switch should zero command");
    }

    {
        ml::ControllerCoreOptions limited_options = options;
        limited_options.max_target_rate = 0.02f;
        ml::ControllerCore core(cfg, limited_options);
        FakeExternalPolicy dance(ml::ControlMode::Dance, "LimitedDance", 0.08f);
        core.register_external_policy("LimitedDance", dance, true);

        const ml::JointArray start_q = cfg.default_motor();
        core.seed_target(start_q);
        const auto out = core.step(
            make_snapshot(start_q),
            ml::Command{},
            ml::mode_request_for_control_mode(ml::ControlMode::Dance, "LimitedDance"),
            cfg.policy_dt);
        require(out.telemetry.mode == ml::ControlMode::Dance, "limited dance mode telemetry");
        require(out.telemetry.policy_evaluated, "limited dance should evaluate policy");

        const ml::JointArray raw_target = offset_target(start_q, 0.08f);
        const float max_delta = limited_options.max_target_rate * cfg.policy_dt;
        require_joint_array_near(out.telemetry.raw_policy_target, raw_target, "limited dance raw target");
        for (int i = 0; i < ml::kNumJoints; ++i) {
            const float raw_delta = raw_target[static_cast<size_t>(i)] - start_q[static_cast<size_t>(i)];
            const float expected =
                start_q[static_cast<size_t>(i)] + std::clamp(raw_delta, -max_delta, max_delta);
            if (!near(out.target.q[static_cast<size_t>(i)], expected)) {
                throw std::runtime_error("external policy target rate limit: joint " + std::to_string(i));
            }
        }
    }
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " /path/to/LocoMode.yaml\n";
        return 2;
    }

    try {
        check_stand_passive_final_modes(argv[1]);
        check_runtime_adapter_flow(argv[1]);
        check_shared_target_rate_limit(argv[1]);
        check_shared_motion_safety(argv[1]);
        check_external_policy_flow(argv[1]);
    } catch (const std::exception& error) {
        std::cerr << "[controller_core_check][FAIL] " << error.what() << "\n";
        return 1;
    }

    std::cout << "[controller_core_check] PASS\n";
    return 0;
}
