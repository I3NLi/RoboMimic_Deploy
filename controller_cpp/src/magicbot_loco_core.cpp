#include "magicbot_loco_core.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <ifaddrs.h>
#include <limits>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>

#include <arpa/inet.h>
#include <yaml-cpp/yaml.h>

namespace magicbot_loco {
namespace {

constexpr std::array<const char*, kNumJoints> kSdkJointNames{
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "head_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
};

constexpr std::array<std::array<float, 2>, kNumJoints> kJointLimits{{
    {{-2.7925f, 2.7925f}},
    {{-0.524f, 2.967f}},
    {{-2.7925f, 2.7925f}},
    {{0.0f, 2.653f}},
    {{-0.873f, 0.524f}},
    {{-0.262f, 0.262f}},
    {{-2.7925f, 2.7925f}},
    {{-2.967f, 0.524f}},
    {{-2.7925f, 2.7925f}},
    {{0.0f, 2.653f}},
    {{-0.873f, 0.524f}},
    {{-0.262f, 0.262f}},
    {{-2.79f, 2.79f}},
    {{-0.6981f, 0.6981f}},
    {{-2.88f, 2.88f}},
    {{-0.174533f, 2.18166f}},
    {{-2.618f, 2.618f}},
    {{-0.959931f, 1.5708f}},
    {{-1.5708f, 1.5708f}},
    {{-2.88f, 2.88f}},
    {{-2.18166f, 0.174533f}},
    {{-2.618f, 2.618f}},
    {{-0.959931f, 1.5708f}},
    {{-1.5708f, 1.5708f}},
}};

Vec read_fvec(const YAML::Node& cfg, const char* key)
{
    if (!cfg[key]) {
        throw std::runtime_error(std::string("missing YAML key: ") + key);
    }
    Vec out;
    if (cfg[key].IsSequence()) {
        for (const auto& v : cfg[key]) out.push_back(v.as<float>());
    } else {
        out.push_back(cfg[key].as<float>());
    }
    return out;
}

IVec read_ivec(const YAML::Node& cfg, const char* key)
{
    if (!cfg[key]) {
        throw std::runtime_error(std::string("missing YAML key: ") + key);
    }
    IVec out;
    for (const auto& v : cfg[key]) out.push_back(v.as<int>());
    return out;
}

void require_size(const Vec& v, int expected, const char* name)
{
    if (static_cast<int>(v.size()) != expected) {
        std::ostringstream oss;
        oss << name << " length " << v.size() << " must be " << expected;
        throw std::runtime_error(oss.str());
    }
}

std::filesystem::path resolve_policy_path(
    const std::filesystem::path& config_path,
    const std::filesystem::path& raw_path)
{
    if (raw_path.is_absolute()) return raw_path;
    const auto config_dir = config_path.parent_path();
    std::vector<std::filesystem::path> candidates{
        config_dir / raw_path,
        config_dir.parent_path() / raw_path,
        config_dir.parent_path() / "model" / raw_path.filename(),
    };
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) return std::filesystem::absolute(candidate);
    }
    return std::filesystem::absolute(candidates.front());
}

bool finite_all(const JointArray& a)
{
    for (float v : a) {
        if (!std::isfinite(v)) return false;
    }
    return true;
}

template <typename ArrayT>
bool finite_all_small(const ArrayT& a)
{
    for (float v : a) {
        if (!std::isfinite(v)) return false;
    }
    return true;
}

float norm3(const std::array<float, 3>& a)
{
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

}  // namespace

JointArray LocoConfig::kps_motor() const
{
    return reorder_lab_to_motor(kps_lab, policy_to_motor);
}

JointArray LocoConfig::kds_motor() const
{
    return reorder_lab_to_motor(kds_lab, policy_to_motor);
}

JointArray LocoConfig::tau_limit_motor() const
{
    return reorder_lab_to_motor(tau_limit_lab, policy_to_motor);
}

JointArray LocoConfig::default_motor() const
{
    return reorder_lab_to_motor(default_lab, policy_to_motor);
}

LocoConfig load_loco_config(const std::filesystem::path& path)
{
    const auto config_path = std::filesystem::absolute(path);
    YAML::Node raw = YAML::LoadFile(config_path.string());
    LocoConfig cfg;
    cfg.config_path = config_path;
    cfg.num_actions = raw["num_actions"].as<int>();
    cfg.command_dim = raw["command_dim"].as<int>();
    cfg.num_obs = raw["num_obs"].as<int>();

    if (cfg.command_dim != 3 && cfg.command_dim != 4) {
        throw std::runtime_error("command_dim must be 3 or 4");
    }
    const int base_obs = 6 + cfg.command_dim + 3 * cfg.num_actions;
    cfg.gait_phase_dim = raw["gait_phase_dim"] ? raw["gait_phase_dim"].as<int>() : std::max(0, cfg.num_obs - base_obs);
    const int expected_obs = base_obs + cfg.gait_phase_dim;
    if (cfg.num_actions != kNumJoints) {
        throw std::runtime_error("num_actions must be 24");
    }
    if (cfg.gait_phase_dim != 0 && cfg.gait_phase_dim != 2) {
        throw std::runtime_error("gait_phase_dim must be 0 or 2");
    }
    if (cfg.num_obs != expected_obs) {
        std::ostringstream oss;
        oss << "num_obs=" << cfg.num_obs << ", expected " << expected_obs;
        throw std::runtime_error(oss.str());
    }

    cfg.gait_phase_period = raw["gait_phase_period"] ? raw["gait_phase_period"].as<float>() : 0.6f;
    cfg.gait_phase_stand_threshold =
        raw["gait_phase_stand_threshold"] ? raw["gait_phase_stand_threshold"].as<float>() : 0.02f;
    cfg.policy_dt = raw["policy_dt"] ? raw["policy_dt"].as<float>() : static_cast<float>(kDefaultPolicyDt);
    if (cfg.gait_phase_period <= 0.0f) throw std::runtime_error("gait_phase_period must be > 0");
    if (cfg.policy_dt <= 0.0f) throw std::runtime_error("policy_dt must be > 0");

    cfg.policy_path = resolve_policy_path(config_path, raw["policy_path"].as<std::string>());
    cfg.policy_to_motor = read_ivec(raw, "joint2motor_idx");
    if (static_cast<int>(cfg.policy_to_motor.size()) != cfg.num_actions) {
        throw std::runtime_error("joint2motor_idx length must match num_actions");
    }
    IVec sorted = cfg.policy_to_motor;
    std::sort(sorted.begin(), sorted.end());
    for (int i = 0; i < cfg.num_actions; ++i) {
        if (sorted[i] != i) throw std::runtime_error("joint2motor_idx must be a permutation of 0..23");
    }

    cfg.kps_lab = read_fvec(raw, "kps");
    cfg.kds_lab = read_fvec(raw, "kds");
    cfg.tau_limit_lab = read_fvec(raw, "tau_limit");
    cfg.default_lab = read_fvec(raw, "default_angles");
    require_size(cfg.kps_lab, cfg.num_actions, "kps");
    require_size(cfg.kds_lab, cfg.num_actions, "kds");
    require_size(cfg.tau_limit_lab, cfg.num_actions, "tau_limit");
    require_size(cfg.default_lab, cfg.num_actions, "default_angles");

    cfg.cmd_scale = read_fvec(raw, "cmd_scale");
    require_size(cfg.cmd_scale, cfg.command_dim, "cmd_scale");
    if (raw["cmd_deadzone"]) {
        if (raw["cmd_deadzone"].IsSequence()) {
            Vec dz = read_fvec(raw, "cmd_deadzone");
            if (dz.size() == 1) dz.assign(3, dz[0]);
            if (dz.size() != 3) throw std::runtime_error("cmd_deadzone must be scalar or length 3");
            for (int i = 0; i < 3; ++i) cfg.cmd_deadzone[i] = dz[i];
        } else {
            cfg.cmd_deadzone.fill(raw["cmd_deadzone"].as<float>());
        }
    }

    auto cmd_range = raw["cmd_range"];
    cfg.cmd_range.vx = {cmd_range["lin_vel_x"][0].as<float>(), cmd_range["lin_vel_x"][1].as<float>()};
    cfg.cmd_range.vy = {cmd_range["lin_vel_y"][0].as<float>(), cmd_range["lin_vel_y"][1].as<float>()};
    cfg.cmd_range.wz = {cmd_range["ang_vel_z"][0].as<float>(), cmd_range["ang_vel_z"][1].as<float>()};

    cfg.root_height_command = raw["root_height_command"].as<float>();
    cfg.obs_clip = raw["obs_clip"] ? raw["obs_clip"].as<float>() : 100.0f;
    cfg.ang_vel_scale = raw["ang_vel_scale"].as<float>();
    cfg.dof_pos_scale = raw["dof_pos_scale"].as<float>();
    cfg.dof_vel_scale = raw["dof_vel_scale"].as<float>();
    cfg.tau_limit_scale = raw["tau_limit_scale"] ? raw["tau_limit_scale"].as<float>() : 1.0f;

    cfg.action_scale = read_fvec(raw, "action_scale");
    if (cfg.action_scale.size() == 1) {
        cfg.action_scale.assign(cfg.num_actions, cfg.action_scale[0]);
    }
    require_size(cfg.action_scale, cfg.num_actions, "action_scale");
    return cfg;
}

std::array<float, 3> gravity_orientation(const std::array<float, 4>& quat_wxyz)
{
    const float qw = quat_wxyz[0];
    const float qx = quat_wxyz[1];
    const float qy = quat_wxyz[2];
    const float qz = quat_wxyz[3];
    return {
        2.0f * (-qz * qx + qw * qy),
        -2.0f * (qz * qy + qw * qx),
        1.0f - 2.0f * (qw * qw + qz * qz),
    };
}

Vec apply_deadzone(const std::array<float, 3>& raw_cmd, const std::array<float, 3>& deadzone)
{
    Vec out{raw_cmd[0], raw_cmd[1], raw_cmd[2]};
    for (int i = 0; i < 3; ++i) {
        const float dz = std::clamp(deadzone[i], 0.0f, 0.95f);
        const float v = out[i];
        if (std::fabs(v) <= dz) {
            out[i] = 0.0f;
        } else {
            out[i] = std::copysign((std::fabs(v) - dz) / std::max(1e-6f, 1.0f - dz), v);
        }
    }
    return out;
}

Vec scale_command(const Vec& deadzone_cmd, const LocoConfig& cfg, bool apply_obs_scale)
{
    Vec cmd(static_cast<size_t>(cfg.command_dim), 0.0f);
    const std::array<std::array<float, 2>, 3> ranges{cfg.cmd_range.vx, cfg.cmd_range.vy, cfg.cmd_range.wz};
    for (int i = 0; i < 3 && i < cfg.command_dim; ++i) {
        const float lo = ranges[i][0];
        const float hi = ranges[i][1];
        const float value = i < static_cast<int>(deadzone_cmd.size()) ? deadzone_cmd[i] : 0.0f;
        if (lo < 0.0f && hi > 0.0f) {
            cmd[i] = value * (value >= 0.0f ? hi : std::fabs(lo));
        } else {
            cmd[i] = (value + 1.0f) * (hi - lo) * 0.5f + lo;
        }
    }
    if (cfg.command_dim >= 4) cmd[3] = cfg.root_height_command;
    if (apply_obs_scale) {
        for (int i = 0; i < cfg.command_dim; ++i) cmd[i] *= cfg.cmd_scale[i];
    }
    return cmd;
}

JointArray reorder_lab_to_motor(const Vec& values_lab, const IVec& policy_to_motor)
{
    JointArray out{};
    for (size_t lab_idx = 0; lab_idx < policy_to_motor.size() && lab_idx < values_lab.size(); ++lab_idx) {
        const int motor_idx = policy_to_motor[lab_idx];
        if (motor_idx >= 0 && motor_idx < kNumJoints) out[static_cast<size_t>(motor_idx)] = values_lab[lab_idx];
    }
    return out;
}

JointArray torque_limited_target(
    const JointArray& target_motor,
    const JointArray& q_motor,
    const JointArray& dq_motor,
    const JointArray& kp_motor,
    const JointArray& kd_motor,
    const JointArray& tau_limit_motor,
    float tau_limit_scale)
{
    JointArray limited{};
    for (int i = 0; i < kNumJoints; ++i) {
        const float kp = kp_motor[i];
        const float kd = kd_motor[i];
        float tau = (target_motor[i] - q_motor[i]) * kp + (0.0f - dq_motor[i]) * kd;
        const float tau_limit = std::max(0.0f, tau_limit_motor[i] * tau_limit_scale);
        tau = std::clamp(tau, -tau_limit, tau_limit);
        if (kp > 1e-6f) {
            limited[i] = q_motor[i] + (tau + kd * dq_motor[i]) / kp;
        } else {
            limited[i] = target_motor[i];
        }
    }
    return limited;
}

JointArray clamp_and_rate_limit(
    const JointArray& target,
    const JointArray& previous,
    float max_rate,
    float dt,
    float joint_limit_margin)
{
    JointArray out{};
    const float max_delta = std::max(0.0f, max_rate * dt);
    for (int i = 0; i < kNumJoints; ++i) {
        const float lower = kJointLimits[i][0] + joint_limit_margin;
        const float upper = kJointLimits[i][1] - joint_limit_margin;
        const float clipped = std::clamp(target[i], lower, upper);
        out[i] = previous[i] + std::clamp(clipped - previous[i], -max_delta, max_delta);
    }
    return out;
}

RateWatchdog::RateWatchdog(RateWatchdogConfig cfg)
    : cfg_(cfg),
      window_start_(std::chrono::steady_clock::now()),
      last_tick_(window_start_)
{
    if (cfg_.window_s < 0.05) cfg_.window_s = 0.05;
    if (cfg_.max_gap_s < 0.0) cfg_.max_gap_s = 0.0;
}

void RateWatchdog::check()
{
    if (!cfg_.enabled) return;
    const auto now = std::chrono::steady_clock::now();
    if (have_last_) {
        const double gap = std::chrono::duration<double>(now - last_tick_).count();
        if (cfg_.max_gap_s > 0.0 && gap > cfg_.max_gap_s) {
            std::ostringstream oss;
            oss << "control loop gap " << gap * 1000.0 << "ms exceeded " << cfg_.max_gap_s * 1000.0 << "ms";
            throw std::runtime_error(oss.str());
        }
    }
    have_last_ = true;
    last_tick_ = now;

    ++window_count_;
    const double elapsed = std::chrono::duration<double>(now - window_start_).count();
    if (elapsed < cfg_.window_s) return;
    const double hz = static_cast<double>(window_count_) / std::max(elapsed, 1e-9);
    if (hz < cfg_.min_hz) {
        std::ostringstream oss;
        oss << "control rate " << hz << "Hz below safety floor " << cfg_.min_hz << "Hz over " << elapsed << "s";
        throw std::runtime_error(oss.str());
    }
    window_start_ = now;
    window_count_ = 0;
}

OnnxLocoPolicy::OnnxLocoPolicy(const LocoConfig& cfg)
    : cfg_(cfg),
      env_(ORT_LOGGING_LEVEL_WARNING, "magicbot_z1_loco")
{
    if (!std::filesystem::exists(cfg_.policy_path)) {
        throw std::runtime_error("ONNX policy not found: " + cfg_.policy_path.string());
    }
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = std::make_unique<Ort::Session>(env_, cfg_.policy_path.string().c_str(), session_options_);

    Ort::AllocatorWithDefaultOptions alloc;
    const size_t n_inputs = session_->GetInputCount();
    const size_t n_outputs = session_->GetOutputCount();
    input_names_owned_.reserve(n_inputs);
    output_names_owned_.reserve(n_outputs);
    for (size_t i = 0; i < n_inputs; ++i) {
        auto name = session_->GetInputNameAllocated(i, alloc);
        input_names_owned_.push_back(name ? std::string(name.get()) : std::string());
    }
    for (size_t i = 0; i < n_outputs; ++i) {
        auto name = session_->GetOutputNameAllocated(i, alloc);
        output_names_owned_.push_back(name ? std::string(name.get()) : std::string());
    }
    for (auto& name : input_names_owned_) input_names_.push_back(name.c_str());
    for (auto& name : output_names_owned_) output_names_.push_back(name.c_str());

    if (n_inputs != 1 || n_outputs < 1) {
        throw std::runtime_error("expected one ONNX input and at least one output");
    }
    const auto input_shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (!input_shape.empty()) {
        const int64_t input_dim = input_shape.back();
        if (input_dim > 0 && input_dim != cfg_.num_obs) {
            std::ostringstream oss;
            oss << "ONNX input dim " << input_dim << " does not match num_obs " << cfg_.num_obs;
            throw std::runtime_error(oss.str());
        }
    }
    const auto output_shape = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (!output_shape.empty()) {
        const int64_t output_dim = output_shape.back();
        if (output_dim > 0 && output_dim != cfg_.num_actions) {
            std::ostringstream oss;
            oss << "ONNX output dim " << output_dim << " does not match num_actions " << cfg_.num_actions;
            throw std::runtime_error(oss.str());
        }
    }
    last_action_lab_.assign(static_cast<size_t>(cfg_.num_actions), 0.0f);
}

void OnnxLocoPolicy::reset()
{
    std::fill(last_action_lab_.begin(), last_action_lab_.end(), 0.0f);
    gait_phase_time_ = 0.0f;
}

void OnnxLocoPolicy::warmup(int rounds)
{
    JointArray q = cfg_.default_motor();
    JointArray dq{};
    std::array<float, 3> zero3{0.0f, 0.0f, 0.0f};
    std::array<float, 3> gravity{0.0f, 0.0f, -1.0f};
    for (int i = 0; i < std::max(1, rounds); ++i) {
        (void)infer(q, dq, zero3, gravity, zero3);
    }
    reset();
}

std::array<float, 2> OnnxLocoPolicy::gait_phase_from_command(const Vec& cmd_unscaled)
{
    std::array<float, 2> mask{1.0f, 1.0f};
    if (cfg_.gait_phase_dim <= 0) return mask;

    const float vx = cmd_unscaled.size() > 0 ? cmd_unscaled[0] : 0.0f;
    const float vy = cmd_unscaled.size() > 1 ? cmd_unscaled[1] : 0.0f;
    const float wz = cmd_unscaled.size() > 2 ? cmd_unscaled[2] : 0.0f;
    const float command_norm = std::sqrt(vx * vx + vy * vy + wz * wz);
    if (command_norm >= cfg_.gait_phase_stand_threshold) {
        constexpr float kTwoPi = 6.2831853071795864769f;
        const float phase = std::fmod(gait_phase_time_, cfg_.gait_phase_period) / cfg_.gait_phase_period;
        const float sin_pos = std::sin(kTwoPi * phase);
        mask[0] = sin_pos >= 0.0f ? 1.0f : 0.0f;
        mask[1] = sin_pos < 0.0f ? 1.0f : 0.0f;
    }
    gait_phase_time_ = std::fmod(gait_phase_time_ + cfg_.policy_dt, cfg_.gait_phase_period);
    return mask;
}

InferResult OnnxLocoPolicy::infer(
    const JointArray& q_motor,
    const JointArray& dq_motor,
    const std::array<float, 3>& ang_vel,
    const std::array<float, 3>& projected_gravity,
    const std::array<float, 3>& raw_cmd)
{
    Vec deadzone_cmd = apply_deadzone(raw_cmd, cfg_.cmd_deadzone);
    Vec cmd_unscaled = scale_command(deadzone_cmd, cfg_, false);
    Vec cmd = cmd_unscaled;
    for (int i = 0; i < cfg_.command_dim; ++i) cmd[i] *= cfg_.cmd_scale[i];

    Vec obs(static_cast<size_t>(cfg_.num_obs), 0.0f);
    obs[0] = ang_vel[0] * cfg_.ang_vel_scale;
    obs[1] = ang_vel[1] * cfg_.ang_vel_scale;
    obs[2] = ang_vel[2] * cfg_.ang_vel_scale;
    obs[3] = projected_gravity[0];
    obs[4] = projected_gravity[1];
    obs[5] = projected_gravity[2];
    const int cmd_end = 6 + cfg_.command_dim;
    for (int i = 0; i < cfg_.command_dim; ++i) obs[6 + i] = cmd[i];

    for (int lab_idx = 0; lab_idx < cfg_.num_actions; ++lab_idx) {
        const int motor_idx = cfg_.policy_to_motor[lab_idx];
        const float q_obs = (q_motor[motor_idx] - cfg_.default_lab[lab_idx]) * cfg_.dof_pos_scale;
        const float dq_obs = dq_motor[motor_idx] * cfg_.dof_vel_scale;
        obs[cmd_end + lab_idx] = q_obs;
        obs[cmd_end + cfg_.num_actions + lab_idx] = dq_obs;
        obs[cmd_end + 2 * cfg_.num_actions + lab_idx] = last_action_lab_[lab_idx];
    }
    if (cfg_.gait_phase_dim == 2) {
        const auto gait = gait_phase_from_command(cmd_unscaled);
        const int gait_start = cmd_end + 3 * cfg_.num_actions;
        obs[gait_start] = gait[0];
        obs[gait_start + 1] = gait[1];
    }
    for (float& v : obs) {
        if (!std::isfinite(v)) v = 0.0f;
        if (cfg_.obs_clip > 0.0f) v = std::clamp(v, -cfg_.obs_clip, cfg_.obs_clip);
    }

    auto mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    std::array<int64_t, 2> shape{1, static_cast<int64_t>(cfg_.num_obs)};
    Ort::Value input = Ort::Value::CreateTensor<float>(mem, obs.data(), obs.size(), shape.data(), shape.size());
    auto outputs = session_->Run(
        Ort::RunOptions{nullptr},
        input_names_.data(),
        &input,
        1,
        output_names_.data(),
        output_names_.size());

    const float* out_ptr = outputs[0].GetTensorData<float>();
    const size_t out_count = outputs[0].GetTensorTypeAndShapeInfo().GetElementCount();
    if (static_cast<int>(out_count) != cfg_.num_actions) {
        throw std::runtime_error("policy output dimension does not match num_actions");
    }

    Vec action_lab(static_cast<size_t>(cfg_.num_actions), 0.0f);
    Vec target_lab(static_cast<size_t>(cfg_.num_actions), 0.0f);
    for (int i = 0; i < cfg_.num_actions; ++i) {
        const float raw = std::isfinite(out_ptr[i]) ? out_ptr[i] : 0.0f;
        action_lab[i] = std::clamp(raw, -100.0f, 100.0f);
        target_lab[i] = action_lab[i] * cfg_.action_scale[i] + cfg_.default_lab[i];
    }
    last_action_lab_ = action_lab;
    return InferResult{action_lab, reorder_lab_to_motor(target_lab, cfg_.policy_to_motor)};
}

MotionSafety::MotionSafety(SafetyConfig cfg, const LocoConfig& loco_cfg)
    : cfg_(std::move(cfg)),
      loco_cfg_(loco_cfg),
      default_motor_(loco_cfg.default_motor())
{
}

std::vector<int> MotionSafety::scoped_joint_ids() const
{
    std::vector<int> ids;
    if (cfg_.joint_scope == "all") {
        for (int i = 0; i < kNumJoints; ++i) ids.push_back(i);
    } else if (cfg_.joint_scope == "legs") {
        for (int i = 0; i < 12; ++i) ids.push_back(i);
    } else {
        for (int i = 0; i < 13; ++i) ids.push_back(i);
    }
    return ids;
}

void MotionSafety::check(
    const RobotSnapshot& state,
    const JointArray* command_target,
    const JointArray* raw_target,
    const JointArray* previous_raw_target) const
{
    if (!cfg_.enabled) return;
    if (!finite_all(state.q) || !finite_all(state.dq) || !finite_all_small(state.quat) ||
        !finite_all_small(state.ang_vel)) {
        throw std::runtime_error("motion safety: state contains non-finite values");
    }
    const auto ids = scoped_joint_ids();

    if (cfg_.max_joint_vel > 0.0f) {
        int idx = ids.front();
        float max_abs = 0.0f;
        for (int id : ids) {
            const float v = std::fabs(state.dq[id]);
            if (v > max_abs) {
                max_abs = v;
                idx = id;
            }
        }
        if (max_abs > cfg_.max_joint_vel) {
            std::ostringstream oss;
            oss << "motion safety: joint velocity " << max_abs << "rad/s on " << joint_name(idx)
                << " exceeded " << cfg_.max_joint_vel;
            throw std::runtime_error(oss.str());
        }
    }

    if (cfg_.max_ang_vel > 0.0f) {
        const float body_w = norm3(state.ang_vel);
        if (body_w > cfg_.max_ang_vel) {
            std::ostringstream oss;
            oss << "motion safety: body angular velocity " << body_w << "rad/s exceeded " << cfg_.max_ang_vel;
            throw std::runtime_error(oss.str());
        }
    }

    if (cfg_.max_gravity_xy > 0.0f) {
        const auto g = gravity_orientation(state.quat);
        const float gxy = std::sqrt(g[0] * g[0] + g[1] * g[1]);
        if (gxy > cfg_.max_gravity_xy) {
            std::ostringstream oss;
            oss << "motion safety: projected gravity xy " << gxy << " exceeded " << cfg_.max_gravity_xy;
            throw std::runtime_error(oss.str());
        }
    }

    if (cfg_.max_default_dev > 0.0f) {
        int idx = ids.front();
        float max_abs = 0.0f;
        for (int id : ids) {
            const float v = std::fabs(state.q[id] - default_motor_[id]);
            if (v > max_abs) {
                max_abs = v;
                idx = id;
            }
        }
        if (max_abs > cfg_.max_default_dev) {
            std::ostringstream oss;
            oss << "motion safety: joint deviation " << max_abs << "rad on " << joint_name(idx)
                << " exceeded " << cfg_.max_default_dev;
            throw std::runtime_error(oss.str());
        }
    }

    if (command_target != nullptr) {
        if (!finite_all(*command_target)) throw std::runtime_error("motion safety: command target non-finite");
        if (cfg_.max_target_error > 0.0f) {
            int idx = ids.front();
            float max_abs = 0.0f;
            for (int id : ids) {
                const float v = std::fabs((*command_target)[id] - state.q[id]);
                if (v > max_abs) {
                    max_abs = v;
                    idx = id;
                }
            }
            if (max_abs > cfg_.max_target_error) {
                std::ostringstream oss;
                oss << "motion safety: target error " << max_abs << "rad on " << joint_name(idx)
                    << " exceeded " << cfg_.max_target_error;
                throw std::runtime_error(oss.str());
            }
        }
    }

    if (raw_target != nullptr) {
        if (!finite_all(*raw_target)) throw std::runtime_error("motion safety: raw policy target non-finite");
        if (cfg_.max_policy_target_dev > 0.0f) {
            int idx = ids.front();
            float max_abs = 0.0f;
            for (int id : ids) {
                const float v = std::fabs((*raw_target)[id] - default_motor_[id]);
                if (v > max_abs) {
                    max_abs = v;
                    idx = id;
                }
            }
            if (max_abs > cfg_.max_policy_target_dev) {
                std::ostringstream oss;
                oss << "motion safety: policy target deviation " << max_abs << "rad on " << joint_name(idx)
                    << " exceeded " << cfg_.max_policy_target_dev;
                throw std::runtime_error(oss.str());
            }
        }
        if (previous_raw_target != nullptr && cfg_.max_policy_target_jump > 0.0f) {
            if (!finite_all(*previous_raw_target)) {
                throw std::runtime_error("motion safety: previous raw policy target non-finite");
            }
            int idx = ids.front();
            float max_abs = 0.0f;
            for (int id : ids) {
                const float v = std::fabs((*raw_target)[id] - (*previous_raw_target)[id]);
                if (v > max_abs) {
                    max_abs = v;
                    idx = id;
                }
            }
            if (max_abs > cfg_.max_policy_target_jump) {
                std::ostringstream oss;
                oss << "motion safety: policy target jump " << max_abs << "rad on " << joint_name(idx)
                    << " exceeded " << cfg_.max_policy_target_jump;
                throw std::runtime_error(oss.str());
            }
        }
    }
}

std::string joint_name(int sdk_idx)
{
    if (sdk_idx < 0 || sdk_idx >= kNumJoints) return "unknown_joint";
    return kSdkJointNames[static_cast<size_t>(sdk_idx)];
}

std::string counts_string(const Counts& counts)
{
    std::ostringstream oss;
    oss << "{leg=" << counts.leg << ", arm=" << counts.arm << ", waist=" << counts.waist
        << ", head=" << counts.head << "}";
    return oss.str();
}

bool local_ip_exists(const std::string& local_ip)
{
    if (local_ip.empty()) return false;
    ifaddrs* ifaddr = nullptr;
    if (getifaddrs(&ifaddr) == -1) return false;
    bool found = false;
    for (ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) continue;
        char host[INET_ADDRSTRLEN] = {0};
        const auto* addr = reinterpret_cast<sockaddr_in*>(ifa->ifa_addr);
        if (inet_ntop(AF_INET, &addr->sin_addr, host, sizeof(host)) != nullptr && local_ip == host) {
            found = true;
            break;
        }
    }
    freeifaddrs(ifaddr);
    return found;
}

}  // namespace magicbot_loco
