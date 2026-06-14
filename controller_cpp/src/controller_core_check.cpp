#include "controller_runtime.h"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace ml = magicbot_loco;

namespace {

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

    runtime.write_damping(4.5f);
    require(adapter.damping_writes == 1, "runtime write_damping should call adapter");
    require(near(adapter.last_damping_kd, 4.5f), "runtime write_damping kd");
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
    } catch (const std::exception& error) {
        std::cerr << "[controller_core_check][FAIL] " << error.what() << "\n";
        return 1;
    }

    std::cout << "[controller_core_check] PASS\n";
    return 0;
}
