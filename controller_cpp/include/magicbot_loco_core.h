#pragma once

#include <array>
#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace magicbot_loco {

constexpr int kNumJoints = 24;
constexpr double kControlHz = 500.0;
constexpr double kControlDt = 1.0 / kControlHz;
constexpr double kDefaultPolicyDt = 0.02;

using Vec = std::vector<float>;
using IVec = std::vector<int>;
using JointArray = std::array<float, kNumJoints>;

struct Counts {
    int leg{0};
    int arm{0};
    int waist{0};
    int head{0};
};

struct RobotSnapshot {
    JointArray q{};
    JointArray dq{};
    std::array<float, 4> quat{1.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 3> ang_vel{0.0f, 0.0f, 0.0f};
    Counts counts{};
};

struct CommandRange {
    std::array<float, 2> vx{-1.0f, 1.0f};
    std::array<float, 2> vy{-1.0f, 1.0f};
    std::array<float, 2> wz{-1.0f, 1.0f};
};

struct LocoConfig {
    std::filesystem::path config_path;
    std::filesystem::path policy_path;
    int command_dim{4};
    int num_actions{kNumJoints};
    int num_obs{82};
    IVec policy_to_motor;
    Vec kps_lab;
    Vec kds_lab;
    Vec tau_limit_lab;
    Vec default_lab;
    Vec cmd_scale;
    std::array<float, 3> cmd_deadzone{0.0f, 0.0f, 0.0f};
    std::array<float, 3> cmd_slew_rate{0.0f, 0.0f, 0.0f};
    CommandRange cmd_range;
    float root_height_command{0.69f};
    float obs_clip{100.0f};
    int gait_phase_dim{0};
    float gait_phase_period{0.6f};
    float gait_phase_stand_threshold{0.02f};
    float policy_dt{kDefaultPolicyDt};
    Vec action_scale;
    float ang_vel_scale{1.0f};
    float dof_pos_scale{1.0f};
    float dof_vel_scale{1.0f};
    float tau_limit_scale{1.0f};

    JointArray kps_motor() const;
    JointArray kds_motor() const;
    JointArray tau_limit_motor() const;
    JointArray default_motor() const;
};

struct InferResult {
    Vec action_lab;
    JointArray target_motor{};
};

struct SafetyConfig {
    bool enabled{true};
    std::string joint_scope{"body"};
    float max_joint_vel{6.0f};
    float max_ang_vel{3.0f};
    float max_gravity_xy{0.55f};
    float max_default_dev{0.9f};
    float max_target_error{0.5f};
    float max_policy_target_dev{0.9f};
    float max_policy_target_jump{0.5f};
};

struct RateWatchdogConfig {
    bool enabled{true};
    double min_hz{250.0};
    double window_s{0.5};
    double max_gap_s{0.05};
};

class RateWatchdog {
public:
    explicit RateWatchdog(RateWatchdogConfig cfg);
    void check();

private:
    RateWatchdogConfig cfg_;
    std::chrono::steady_clock::time_point window_start_;
    std::chrono::steady_clock::time_point last_tick_;
    bool have_last_{false};
    int window_count_{0};
};

class OnnxLocoPolicy {
public:
    explicit OnnxLocoPolicy(const LocoConfig& cfg);

    InferResult infer(
        const JointArray& q_motor,
        const JointArray& dq_motor,
        const std::array<float, 3>& ang_vel,
        const std::array<float, 3>& projected_gravity,
        const std::array<float, 3>& raw_cmd);

    void reset();
    void warmup(int rounds);
    const Vec& last_action_lab() const { return last_action_lab_; }

private:
    Vec apply_command_slew(const Vec& target_cmd_unscaled);
    std::array<float, 2> gait_phase_from_command(const Vec& cmd_unscaled);

    const LocoConfig& cfg_;
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<std::string> input_names_owned_;
    std::vector<std::string> output_names_owned_;
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    Vec last_action_lab_;
    Vec smoothed_cmd_unscaled_;
    float gait_phase_time_{0.0f};
};

class MotionSafety {
public:
    MotionSafety(SafetyConfig cfg, const LocoConfig& loco_cfg);

    void check(
        const RobotSnapshot& state,
        const JointArray* command_target = nullptr,
        const JointArray* raw_target = nullptr,
        const JointArray* previous_raw_target = nullptr) const;

private:
    std::vector<int> scoped_joint_ids() const;

    SafetyConfig cfg_;
    const LocoConfig& loco_cfg_;
    JointArray default_motor_{};
};

LocoConfig load_loco_config(const std::filesystem::path& path);

std::array<float, 3> gravity_orientation(const std::array<float, 4>& quat_wxyz);
Vec apply_deadzone(const std::array<float, 3>& raw_cmd, const std::array<float, 3>& deadzone);
Vec scale_command(const Vec& deadzone_cmd, const LocoConfig& cfg, bool apply_obs_scale);
JointArray reorder_lab_to_motor(const Vec& values_lab, const IVec& policy_to_motor);
JointArray torque_limited_target(
    const JointArray& target_motor,
    const JointArray& q_motor,
    const JointArray& dq_motor,
    const JointArray& kp_motor,
    const JointArray& kd_motor,
    const JointArray& tau_limit_motor,
    float tau_limit_scale);
JointArray clamp_and_rate_limit(
    const JointArray& target,
    const JointArray& previous,
    float max_rate,
    float dt,
    float joint_limit_margin);

std::string joint_name(int sdk_idx);
std::string counts_string(const Counts& counts);
bool local_ip_exists(const std::string& local_ip);

}  // namespace magicbot_loco
