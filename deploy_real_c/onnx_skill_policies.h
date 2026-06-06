/**
 * onnx_skill_policies.h
 *
 * Extra C++ policy ports for deploy_real_onnx:
 *  - LocoMode (policy/loco_mode/LocoMode.py)
 *  - Dance/KungFu/Kick/KungFu2 motion policies
 *  - SkillCooldown / SkillCast transition policies
 *
 * This header is intended to be included after deploy_real.cpp defines:
 *   - Z1_NUM_MOTOR
 *   - FSMState / FSMStateName / FSMCommand
 *   - StateAndCmd / PolicyOutput
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

namespace onnx_skill {

static inline std::vector<float> read_fvec(const YAML::Node& n, const char* key)
{
    std::vector<float> out;
    if (!n[key]) return out;
    for (const auto& e : n[key]) out.push_back(e.as<float>());
    return out;
}

static inline std::vector<int> read_ivec(const YAML::Node& n, const char* key)
{
    std::vector<int> out;
    if (!n[key]) return out;
    for (const auto& e : n[key]) out.push_back(e.as<int>());
    return out;
}

static inline std::string resolve_model_path(
    const std::filesystem::path& config_path,
    const std::filesystem::path& rel_or_abs)
{
    if (rel_or_abs.is_absolute()) return rel_or_abs.string();
    const auto current_dir = config_path.parent_path().parent_path();  // policy_xxx/
    const auto from_model = current_dir / "model" / rel_or_abs;
    if (std::filesystem::exists(from_model)) return from_model.string();
    const auto from_config = config_path.parent_path() / rel_or_abs;
    if (std::filesystem::exists(from_config)) return from_config.string();
    return from_model.string();
}

static inline std::vector<std::string> split_csv(const std::string& s)
{
    std::vector<std::string> out;
    size_t i = 0;
    while (i < s.size()) {
        size_t j = s.find(',', i);
        if (j == std::string::npos) j = s.size();
        std::string tok = s.substr(i, j - i);
        size_t b = tok.find_first_not_of(" \t\r\n");
        if (b != std::string::npos) {
            size_t e = tok.find_last_not_of(" \t\r\n");
            out.push_back(tok.substr(b, e - b + 1));
        }
        i = j + 1;
    }
    return out;
}

class OrtPolicyBase {
public:
    explicit OrtPolicyBase(const std::string& tag)
        : env_(ORT_LOGGING_LEVEL_WARNING, tag.c_str()) {}

protected:
    void load_onnx(const std::string& model_path)
    {
        model_path_ = model_path;
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(1);
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_ = std::make_unique<Ort::Session>(env_, model_path_.c_str(), opts);

        Ort::AllocatorWithDefaultOptions alloc;
        const size_t n_inputs = session_->GetInputCount();
        const size_t n_outputs = session_->GetOutputCount();
        input_names_owned_.clear();
        output_names_owned_.clear();
        input_names_.clear();
        output_names_.clear();
        input_names_owned_.reserve(n_inputs);
        output_names_owned_.reserve(n_outputs);
        input_names_.reserve(n_inputs);
        output_names_.reserve(n_outputs);

        for (size_t i = 0; i < n_inputs; i++) {
            auto s = session_->GetInputNameAllocated(i, alloc);
            input_names_owned_.push_back(s ? std::string(s.get()) : std::string());
        }
        for (size_t i = 0; i < n_outputs; i++) {
            auto s = session_->GetOutputNameAllocated(i, alloc);
            output_names_owned_.push_back(s ? std::string(s.get()) : std::string());
        }
        for (auto& n : input_names_owned_) input_names_.push_back(n.c_str());
        for (auto& n : output_names_owned_) output_names_.push_back(n.c_str());
    }

    std::vector<float> run_single_input(std::vector<float>& input_buf, int dim)
    {
        auto mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        std::array<int64_t, 2> shape{1, (int64_t)dim};
        Ort::Value input = Ort::Value::CreateTensor<float>(
            mem, input_buf.data(), input_buf.size(), shape.data(), shape.size());
        auto outputs = session_->Run(
            Ort::RunOptions{nullptr},
            input_names_.data(), &input, 1,
            output_names_.data(), output_names_.size());
        const auto* ptr = outputs[0].GetTensorData<float>();
        auto info = outputs[0].GetTensorTypeAndShapeInfo();
        size_t n = info.GetElementCount();
        std::vector<float> out(n, 0.f);
        for (size_t i = 0; i < n; i++) out[i] = std::isfinite(ptr[i]) ? ptr[i] : 0.f;
        return out;
    }

    std::vector<Ort::Value> run_two_inputs(
        std::vector<float>& obs_buf, int obs_dim, float step_val)
    {
        auto mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        std::array<int64_t, 2> obs_shape{1, (int64_t)obs_dim};
        std::array<int64_t, 2> step_shape{1, 1};
        Ort::Value inputs[2] = {
            Ort::Value::CreateTensor<float>(
                mem, obs_buf.data(), obs_buf.size(), obs_shape.data(), obs_shape.size()),
            Ort::Value::CreateTensor<float>(
                mem, &step_val, 1, step_shape.data(), step_shape.size()),
        };
        return session_->Run(
            Ort::RunOptions{nullptr},
            input_names_.data(), inputs, 2,
            output_names_.data(), output_names_.size());
    }

    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
    std::string model_path_;

    std::vector<std::string> input_names_owned_;
    std::vector<std::string> output_names_owned_;
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
};

class LocoModePolicy final : public FSMState, private OrtPolicyBase {
public:
    LocoModePolicy(StateAndCmd& sc, PolicyOutput& po, const std::string& yaml_path)
        : FSMState(FSMStateName::LOCOMODE, "LocoMode(onnx)", sc, po),
          OrtPolicyBase("LocoMode")
    {
        const auto cfg_path = std::filesystem::absolute(std::filesystem::path(yaml_path));
        YAML::Node cfg = YAML::LoadFile(cfg_path.string());

        std::string model_rel = cfg["policy_path"].as<std::string>();
        load_onnx(resolve_model_path(cfg_path, std::filesystem::path(model_rel)));

        kps_ = read_fvec(cfg, "kps");
        kds_ = read_fvec(cfg, "kds");
        default_angles_ = read_fvec(cfg, "default_angles");
        joint2motor_idx_ = read_ivec(cfg, "joint2motor_idx");
        tau_limit_ = read_fvec(cfg, "tau_limit");
        tau_limit_scale_ = cfg["tau_limit_scale"] ? cfg["tau_limit_scale"].as<float>() : 1.0f;
        num_actions_ = cfg["num_actions"].as<int>();
        num_obs_ = cfg["num_obs"].as<int>();
        ang_vel_scale_ = cfg["ang_vel_scale"].as<float>();
        dof_pos_scale_ = cfg["dof_pos_scale"].as<float>();
        dof_vel_scale_ = cfg["dof_vel_scale"].as<float>();
        action_scale_ = cfg["action_scale"].as<float>();
        cmd_scale_ = read_fvec(cfg, "cmd_scale");
        cmd_init_ = read_fvec(cfg, "cmd_init");
        cmd_deadzone_.assign(3, 0.f);
        if (cfg["cmd_deadzone"]) {
            if (cfg["cmd_deadzone"].IsSequence()) {
                auto dz = read_fvec(cfg, "cmd_deadzone");
                for (size_t i = 0; i < std::min((size_t)3, dz.size()); i++) cmd_deadzone_[i] = dz[i];
            } else {
                float dz = cfg["cmd_deadzone"].as<float>();
                for (int i = 0; i < 3; i++) cmd_deadzone_[i] = dz;
            }
        }

        auto cmd_range = cfg["cmd_range"];
        range_vx_ = {cmd_range["lin_vel_x"][0].as<float>(), cmd_range["lin_vel_x"][1].as<float>()};
        range_vy_ = {cmd_range["lin_vel_y"][0].as<float>(), cmd_range["lin_vel_y"][1].as<float>()};
        range_vz_ = {cmd_range["ang_vel_z"][0].as<float>(), cmd_range["ang_vel_z"][1].as<float>()};

        action_.assign(num_actions_, 0.f);
        obs_.assign(num_obs_, 0.f);
        qj_obs_.assign(num_actions_, 0.f);
        dqj_obs_.assign(num_actions_, 0.f);

        // warmup
        std::vector<float> warm(num_obs_, 0.f);
        for (int i = 0; i < 30; i++) {
            (void)run_single_input(warm, num_obs_);
        }
        printf("[LocoMode] Loaded: %s\n", model_path_.c_str());
    }

    void enter() override
    {
        kps_mj_.assign(num_actions_, 0.f);
        kds_mj_.assign(num_actions_, 0.f);
        tau_limit_mj_.assign(num_actions_, 0.f);
        default_mj_.assign(num_actions_, 0.f);
        for (int i = 0; i < (int)joint2motor_idx_.size(); i++) {
            int m = joint2motor_idx_[i];
            if (m < 0 || m >= num_actions_) continue;
            if (i < (int)kps_.size()) kps_mj_[m] = kps_[i];
            if (i < (int)kds_.size()) kds_mj_[m] = kds_[i];
            if (i < (int)tau_limit_.size()) tau_limit_mj_[m] = tau_limit_[i];
            if (i < (int)default_angles_.size()) default_mj_[m] = default_angles_[i];
        }
    }

    void run() override
    {
        std::array<float, 3> joy = sc_.vel_cmd;
        apply_deadzone(joy);
        auto cmd = scale_cmd(joy);

        for (int i = 0; i < (int)joint2motor_idx_.size() && i < num_actions_; i++) {
            int m = joint2motor_idx_[i];
            qj_obs_[i] = sc_.q[m];
            dqj_obs_[i] = sc_.dq[m];
        }
        for (int i = 0; i < num_actions_; i++) {
            float def = i < (int)default_angles_.size() ? default_angles_[i] : 0.f;
            qj_obs_[i] = (qj_obs_[i] - def) * dof_pos_scale_;
            dqj_obs_[i] = dqj_obs_[i] * dof_vel_scale_;
        }

        obs_.assign(num_obs_, 0.f);
        obs_[0] = sc_.ang_vel[0] * ang_vel_scale_;
        obs_[1] = sc_.ang_vel[1] * ang_vel_scale_;
        obs_[2] = sc_.ang_vel[2] * ang_vel_scale_;
        obs_[3] = sc_.gravity_ori[0];
        obs_[4] = sc_.gravity_ori[1];
        obs_[5] = sc_.gravity_ori[2];
        for (int i = 0; i < 3 && (6 + i) < num_obs_; i++) {
            float s = i < (int)cmd_scale_.size() ? cmd_scale_[i] : 1.f;
            obs_[6 + i] = cmd[i] * s;
        }
        int off = 9;
        for (int i = 0; i < num_actions_ && (off + i) < num_obs_; i++) obs_[off + i] = qj_obs_[i];
        off += num_actions_;
        for (int i = 0; i < num_actions_ && (off + i) < num_obs_; i++) obs_[off + i] = dqj_obs_[i];
        off += num_actions_;
        for (int i = 0; i < num_actions_ && (off + i) < num_obs_; i++) obs_[off + i] = action_[i];

        auto out = run_single_input(obs_, num_obs_);
        for (int i = 0; i < num_actions_ && i < (int)out.size(); i++) {
            action_[i] = std::clamp(out[i], -100.f, 100.f);
        }

        std::vector<float> target_mj(num_actions_, 0.f);
        for (int i = 0; i < num_actions_; i++) {
            float def = i < (int)default_angles_.size() ? default_angles_[i] : 0.f;
            float t = action_[i] * action_scale_ + def;
            int m = i < (int)joint2motor_idx_.size() ? joint2motor_idx_[i] : i;
            if (m >= 0 && m < num_actions_) target_mj[m] = t;
        }

        for (int i = 0; i < num_actions_; i++) {
            float kp = std::max(0.f, kps_mj_[i]);
            float kd = std::max(0.f, kds_mj_[i]);
            float tl = std::max(0.f, tau_limit_mj_[i] * tau_limit_scale_);
            float tau = (target_mj[i] - sc_.q[i]) * kp + (0.f - sc_.dq[i]) * kd;
            tau = std::clamp(tau, -tl, tl);
            if (kp > 1e-6f) {
                target_mj[i] = sc_.q[i] + (tau + kd * sc_.dq[i]) / kp;
            }
        }

        for (int i = 0; i < num_actions_ && i < Z1_NUM_MOTOR; i++) {
            po_.actions[i] = target_mj[i];
            po_.kps[i] = kps_mj_[i];
            po_.kds[i] = kds_mj_[i];
        }
    }

    FSMStateName check_change() override
    {
        if (sc_.skill_cmd == FSMCommand::SKILL_1) return FSMStateName::SKILL_DANCE;
        if (sc_.skill_cmd == FSMCommand::SKILL_2) return FSMStateName::SKILL_KUNGFU;
        if (sc_.skill_cmd == FSMCommand::SKILL_3) return FSMStateName::SKILL_KICK;
        if (sc_.skill_cmd == FSMCommand::SKILL_4) return FSMStateName::SKILL_BEYOND_MIMIC;
        if (sc_.skill_cmd == FSMCommand::SKILL_5) return FSMStateName::JOINT_ZERO_CHECK;
        if (sc_.skill_cmd == FSMCommand::SKILL_6) return FSMStateName::IMU_CALIB;
        if (sc_.skill_cmd == FSMCommand::SKILL_7) return FSMStateName::SKILL_TRACK_MIMIC;
        if (sc_.skill_cmd == FSMCommand::PASSIVE) return FSMStateName::PASSIVE;
        return FSMStateName::LOCOMODE;
    }

private:
    void apply_deadzone(std::array<float, 3>& cmd)
    {
        for (int i = 0; i < 3; i++) {
            float dz = std::clamp(cmd_deadzone_[i], 0.f, 0.95f);
            float v = cmd[i];
            if (std::fabs(v) <= dz) cmd[i] = 0.f;
            else {
                float sgn = v >= 0.f ? 1.f : -1.f;
                cmd[i] = sgn * (std::fabs(v) - dz) / std::max(1e-6f, 1.f - dz);
            }
        }
    }

    std::array<float, 3> scale_cmd(const std::array<float, 3>& cmd) const
    {
        std::array<float, 3> out{0, 0, 0};
        const std::array<std::array<float, 2>, 3> ranges{range_vx_, range_vy_, range_vz_};
        for (int i = 0; i < 3; i++) {
            float lo = ranges[i][0], hi = ranges[i][1], v = cmd[i];
            if (lo < 0.f && hi > 0.f) out[i] = v * (v >= 0.f ? hi : std::fabs(lo));
            else out[i] = (v + 1.f) * (hi - lo) * 0.5f + lo;
        }
        return out;
    }

    int num_actions_{24};
    int num_obs_{96};
    float ang_vel_scale_{1.f};
    float dof_pos_scale_{1.f};
    float dof_vel_scale_{1.f};
    float action_scale_{0.25f};
    float tau_limit_scale_{1.f};

    std::array<float, 2> range_vx_{-0.6f, 1.0f};
    std::array<float, 2> range_vy_{-0.5f, 0.5f};
    std::array<float, 2> range_vz_{-1.57f, 1.57f};

    std::vector<int> joint2motor_idx_;
    std::vector<float> kps_, kds_, tau_limit_, default_angles_;
    std::vector<float> cmd_scale_, cmd_init_, cmd_deadzone_;

    std::vector<float> kps_mj_, kds_mj_, tau_limit_mj_, default_mj_;
    std::vector<float> action_, obs_, qj_obs_, dqj_obs_;
};

class MotionPolicy final : public FSMState, private OrtPolicyBase {
public:
    MotionPolicy(
        FSMStateName state_name,
        const std::string& label,
        StateAndCmd& sc, PolicyOutput& po,
        const std::string& yaml_path,
        bool update_history_before_obs,
        bool clip_action10)
        : FSMState(state_name, label, sc, po),
          OrtPolicyBase(label),
          update_history_before_obs_(update_history_before_obs),
          clip_action10_(clip_action10)
    {
        const auto cfg_path = std::filesystem::absolute(std::filesystem::path(yaml_path));
        YAML::Node cfg = YAML::LoadFile(cfg_path.string());
        std::string model_rel = cfg["onnx_path"].as<std::string>();
        load_onnx(resolve_model_path(cfg_path, std::filesystem::path(model_rel)));

        kps_ = read_fvec(cfg, "kps");
        kds_ = read_fvec(cfg, "kds");
        default_angles_ = read_fvec(cfg, "default_angles");
        motion_dof_index_ = read_ivec(cfg, "dof23_index");
        num_actions_ = cfg["num_actions"].as<int>();
        num_obs_ = cfg["num_obs"].as<int>();           // 380
        history_length_ = cfg["history_length"].as<int>();
        motion_length_ = cfg["motion_length"].as<float>();
        ang_vel_scale_ = cfg["ang_vel_scale"].as<float>();
        dof_pos_scale_ = cfg["dof_pos_scale"].as<float>();
        dof_vel_scale_ = cfg["dof_vel_scale"].as<float>();
        action_scale_ = cfg["action_scale"].as<float>();

        action_.assign(num_actions_, 0.f);
        obs_.assign(num_obs_, 0.f);
        qj_motion_.assign(num_actions_, 0.f);
        dqj_motion_.assign(num_actions_, 0.f);
        ang_vel_buf_.assign(3 * history_length_, 0.f);
        proj_g_buf_.assign(3 * history_length_, 0.f);
        dof_pos_buf_.assign(num_actions_ * history_length_, 0.f);
        dof_vel_buf_.assign(num_actions_ * history_length_, 0.f);
        action_buf_.assign(num_actions_ * history_length_, 0.f);
        phase_buf_.assign(history_length_, 0.f);

        std::vector<float> warm(num_obs_, 0.f);
        for (int i = 0; i < 30; i++) (void)run_single_input(warm, num_obs_);
        printf("[%s] Loaded: %s\n", label.c_str(), model_path_.c_str());
    }

    void enter() override
    {
        counter_step_ = 0;
        ref_phase_ = 0.f;
        std::fill(action_.begin(), action_.end(), 0.f);
        std::fill(obs_.begin(), obs_.end(), 0.f);
        std::fill(ang_vel_buf_.begin(), ang_vel_buf_.end(), 0.f);
        std::fill(proj_g_buf_.begin(), proj_g_buf_.end(), 0.f);
        std::fill(dof_pos_buf_.begin(), dof_pos_buf_.end(), 0.f);
        std::fill(dof_vel_buf_.begin(), dof_vel_buf_.end(), 0.f);
        std::fill(action_buf_.begin(), action_buf_.end(), 0.f);
        std::fill(phase_buf_.begin(), phase_buf_.end(), 0.f);
    }

    void run() override
    {
        std::array<float, 3> g = sc_.gravity_ori;
        std::array<float, 3> w = {
            sc_.ang_vel[0] * ang_vel_scale_,
            sc_.ang_vel[1] * ang_vel_scale_,
            sc_.ang_vel[2] * ang_vel_scale_,
        };

        for (int i = 0; i < num_actions_; i++) {
            int m = i < (int)motion_dof_index_.size() ? motion_dof_index_[i] : i;
            if (m < 0 || m >= Z1_NUM_MOTOR) continue;
            float q = sc_.q[m];
            float dq = sc_.dq[m];
            float def = m < (int)default_angles_.size() ? default_angles_[m] : 0.f;
            qj_motion_[i] = (q - def) * dof_pos_scale_;
            dqj_motion_[i] = dq * dof_vel_scale_;
        }

        if (update_history_before_obs_) {
            push_history(w, g);
        }

        std::vector<float> hist;
        hist.reserve(action_buf_.size() + ang_vel_buf_.size() + dof_pos_buf_.size() +
                     dof_vel_buf_.size() + proj_g_buf_.size() + phase_buf_.size());
        hist.insert(hist.end(), action_buf_.begin(), action_buf_.end());
        hist.insert(hist.end(), ang_vel_buf_.begin(), ang_vel_buf_.end());
        hist.insert(hist.end(), dof_pos_buf_.begin(), dof_pos_buf_.end());
        hist.insert(hist.end(), dof_vel_buf_.begin(), dof_vel_buf_.end());
        hist.insert(hist.end(), proj_g_buf_.begin(), proj_g_buf_.end());
        hist.insert(hist.end(), phase_buf_.begin(), phase_buf_.end());

        obs_.clear();
        obs_.reserve(num_obs_);
        obs_.insert(obs_.end(), action_.begin(), action_.end());
        obs_.insert(obs_.end(), w.begin(), w.end());
        obs_.insert(obs_.end(), qj_motion_.begin(), qj_motion_.end());
        obs_.insert(obs_.end(), dqj_motion_.begin(), dqj_motion_.end());
        obs_.insert(obs_.end(), hist.begin(), hist.end());
        obs_.insert(obs_.end(), g.begin(), g.end());
        obs_.push_back(std::min(ref_phase_, 1.0f));
        if ((int)obs_.size() < num_obs_) obs_.resize(num_obs_, 0.f);
        if ((int)obs_.size() > num_obs_) obs_.resize(num_obs_);

        auto out = run_single_input(obs_, num_obs_);
        for (int i = 0; i < num_actions_ && i < (int)out.size(); i++) {
            float v = out[i];
            if (clip_action10_) v = std::clamp(v, -10.f, 10.f);
            action_[i] = v;
        }

        if (!update_history_before_obs_) {
            push_history(w, g);
        }

        std::array<float, Z1_NUM_MOTOR> target{};
        for (int i = 0; i < Z1_NUM_MOTOR; i++) {
            target[i] = i < (int)default_angles_.size() ? default_angles_[i] : sc_.q[i];
        }
        for (int i = 0; i < num_actions_ && i < (int)action_.size(); i++) {
            int m = i < (int)motion_dof_index_.size() ? motion_dof_index_[i] : i;
            if (m < 0 || m >= Z1_NUM_MOTOR) continue;
            float def = m < (int)default_angles_.size() ? default_angles_[m] : 0.f;
            target[m] = action_[i] * action_scale_ + def;
        }

        for (int i = 0; i < Z1_NUM_MOTOR; i++) {
            po_.actions[i] = target[i];
            po_.kps[i] = i < (int)kps_.size() ? kps_[i] : 0.f;
            po_.kds[i] = i < (int)kds_.size() ? kds_[i] : 0.f;
        }

        counter_step_++;
        float t = counter_step_ * 0.02f;
        ref_phase_ = motion_length_ > 1e-6f ? t / motion_length_ : 0.f;
    }

    FSMStateName check_change() override
    {
        if (sc_.skill_cmd == FSMCommand::LOCO) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::SKILL_COOLDOWN;
        }
        if (sc_.skill_cmd == FSMCommand::PASSIVE) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::PASSIVE;
        }
        if (sc_.skill_cmd == FSMCommand::POS_RESET) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::FIXEDPOSE;
        }
        sc_.skill_cmd = FSMCommand::INVALID;
        return name;
    }

private:
    void roll_insert(std::vector<float>& buf, int stride, const float* x)
    {
        if ((int)buf.size() < stride) return;
        std::memmove(buf.data() + stride, buf.data(), (buf.size() - stride) * sizeof(float));
        for (int i = 0; i < stride; i++) buf[i] = x[i];
    }

    void push_history(const std::array<float, 3>& w, const std::array<float, 3>& g)
    {
        roll_insert(ang_vel_buf_, 3, w.data());
        roll_insert(proj_g_buf_, 3, g.data());
        roll_insert(dof_pos_buf_, num_actions_, qj_motion_.data());
        roll_insert(dof_vel_buf_, num_actions_, dqj_motion_.data());
        roll_insert(action_buf_, num_actions_, action_.data());
        float ph = std::min(ref_phase_, 1.0f);
        roll_insert(phase_buf_, 1, &ph);
    }

    bool update_history_before_obs_{false};
    bool clip_action10_{false};
    int num_actions_{23};
    int num_obs_{380};
    int history_length_{4};
    float motion_length_{1.f};
    float ang_vel_scale_{0.25f};
    float dof_pos_scale_{1.f};
    float dof_vel_scale_{0.05f};
    float action_scale_{0.25f};
    int counter_step_{0};
    float ref_phase_{0.f};

    std::vector<int> motion_dof_index_;
    std::vector<float> kps_, kds_, default_angles_;
    std::vector<float> action_, obs_, qj_motion_, dqj_motion_;
    std::vector<float> ang_vel_buf_, proj_g_buf_, dof_pos_buf_, dof_vel_buf_, action_buf_, phase_buf_;
};

class LowerBody15PolicyBase : public FSMState, protected OrtPolicyBase {
public:
    LowerBody15PolicyBase(
        FSMStateName state_name, const std::string& label,
        StateAndCmd& sc, PolicyOutput& po,
        const std::string& yaml_path)
        : FSMState(state_name, label, sc, po),
          OrtPolicyBase(label)
    {
        const auto cfg_path = std::filesystem::absolute(std::filesystem::path(yaml_path));
        YAML::Node cfg = YAML::LoadFile(cfg_path.string());

        std::string policy_rel = cfg["policy_path"].as<std::string>();
        auto model_path = resolve_model_path(cfg_path, std::filesystem::path(policy_rel));
        if (model_path.size() > 3 && model_path.substr(model_path.size() - 3) == ".pt") {
            std::string onnx_guess = model_path.substr(0, model_path.size() - 3) + ".onnx";
            if (std::filesystem::exists(onnx_guess)) {
                model_path = onnx_guess;
            } else if (!std::filesystem::exists(model_path)) {
                throw std::runtime_error(
                    "Neither TorchScript nor exported ONNX exists: " + model_path);
            }
        }
        load_onnx(model_path);

        kps_ = read_fvec(cfg, "kps");
        kds_ = read_fvec(cfg, "kds");
        default_angles_ = read_fvec(cfg, "default_angles");
        upper_idx_ = read_ivec(cfg, "upper_body_motor_idx");
        lower_idx_ = read_ivec(cfg, "lower_body_motor_idx");
        num_actions_ = cfg["num_actions"].as<int>();  // 15
        num_obs_ = cfg["num_obs"].as<int>();
        ang_vel_scale_ = cfg["ang_vel_scale"].as<float>();
        dof_pos_scale_ = cfg["dof_pos_scale"].as<float>();
        dof_vel_scale_ = cfg["dof_vel_scale"].as<float>();
        action_scale_ = cfg["action_scale"].as<float>();
        total_time_ = cfg["total_time"] ? cfg["total_time"].as<float>() : 1.0f;
        period_ = cfg["period"] ? cfg["period"].as<float>() : 0.8f;

        obs_.assign(num_obs_, 0.f);
        action_.assign(num_actions_, 0.f);
        qj_obs_.assign(num_actions_, 0.f);
        dqj_obs_.assign(num_actions_, 0.f);
        upper_init_.assign(upper_idx_.size(), 0.f);
    }

    void enter() override
    {
        num_step_ = std::max(1, (int)std::round(total_time_ / 0.02f));
        cur_step_ = 0;
        alpha_ = 0.f;
        for (size_t i = 0; i < upper_idx_.size(); i++) upper_init_[i] = sc_.q[upper_idx_[i]];
    }

protected:
    void run_lower_body(bool with_phase)
    {
        for (int i = 0; i < num_actions_ && i < (int)lower_idx_.size(); i++) {
            int m = lower_idx_[i];
            float def = m < (int)default_angles_.size() ? default_angles_[m] : 0.f;
            qj_obs_[i] = (sc_.q[m] - def) * dof_pos_scale_;
            dqj_obs_[i] = sc_.dq[m] * dof_vel_scale_;
        }
        obs_.assign(num_obs_, 0.f);
        obs_[0] = sc_.ang_vel[0] * ang_vel_scale_;
        obs_[1] = sc_.ang_vel[1] * ang_vel_scale_;
        obs_[2] = sc_.ang_vel[2] * ang_vel_scale_;
        obs_[3] = sc_.gravity_ori[0];
        obs_[4] = sc_.gravity_ori[1];
        obs_[5] = sc_.gravity_ori[2];
        int off = 9;
        for (int i = 0; i < num_actions_ && (off + i) < num_obs_; i++) obs_[off + i] = qj_obs_[i];
        off += num_actions_;
        for (int i = 0; i < num_actions_ && (off + i) < num_obs_; i++) obs_[off + i] = dqj_obs_[i];
        off += num_actions_;
        for (int i = 0; i < num_actions_ && (off + i) < num_obs_; i++) obs_[off + i] = action_[i];
        off += num_actions_;
        if (with_phase && (off + 1) < num_obs_) {
            float count = cur_step_ * 0.02f;
            float phase = std::fmod(count, period_) / std::max(1e-6f, period_);
            constexpr float kTwoPi = 6.28318530718f;
            obs_[off + 0] = std::sin(kTwoPi * phase);
            obs_[off + 1] = std::cos(kTwoPi * phase);
        }

        auto out = run_single_input(obs_, num_obs_);
        for (int i = 0; i < num_actions_ && i < (int)out.size(); i++) action_[i] = out[i];

        for (int i = 0; i < num_actions_ && i < (int)lower_idx_.size(); i++) {
            int m = lower_idx_[i];
            float def = m < (int)default_angles_.size() ? default_angles_[m] : 0.f;
            po_.actions[m] = action_[i] * action_scale_ + def;
        }
        for (int i = 0; i < Z1_NUM_MOTOR; i++) {
            po_.kps[i] = i < (int)kps_.size() ? kps_[i] : 0.f;
            po_.kds[i] = i < (int)kds_.size() ? kds_[i] : 0.f;
        }
    }

    void advance_alpha()
    {
        cur_step_++;
        alpha_ = std::min(1.0f, (float)cur_step_ / (float)num_step_);
    }

    int num_actions_{15};
    int num_obs_{56};
    float ang_vel_scale_{0.25f};
    float dof_pos_scale_{1.f};
    float dof_vel_scale_{0.05f};
    float action_scale_{0.25f};
    float total_time_{1.f};
    float period_{0.8f};
    int num_step_{1};
    int cur_step_{0};
    float alpha_{0.f};

    std::vector<int> upper_idx_;
    std::vector<int> lower_idx_;
    std::vector<float> kps_, kds_, default_angles_;
    std::vector<float> upper_init_;
    std::vector<float> obs_, action_, qj_obs_, dqj_obs_;
};

class SkillCooldownPolicy final : public LowerBody15PolicyBase {
public:
    SkillCooldownPolicy(StateAndCmd& sc, PolicyOutput& po, const std::string& yaml_path)
        : LowerBody15PolicyBase(FSMStateName::SKILL_COOLDOWN, "SkillCooldown(onnx)", sc, po, yaml_path)
    {
        printf("[SkillCooldown] Loaded: %s\n", model_path_.c_str());
    }

    void run() override
    {
        run_lower_body(true);
        advance_alpha();
        for (size_t j = 0; j < upper_idx_.size(); j++) {
            int m = upper_idx_[j];
            float target = m < (int)default_angles_.size() ? default_angles_[m] : 0.f;
            po_.actions[m] = upper_init_[j] * (1.f - alpha_) + target * alpha_;
        }
    }

    FSMStateName check_change() override
    {
        if (cur_step_ >= num_step_) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::LOCOMODE;
        }
        if (sc_.skill_cmd == FSMCommand::PASSIVE) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::PASSIVE;
        }
        sc_.skill_cmd = FSMCommand::INVALID;
        return FSMStateName::SKILL_COOLDOWN;
    }
};

class SkillCastPolicy final : public LowerBody15PolicyBase {
public:
    SkillCastPolicy(StateAndCmd& sc, PolicyOutput& po, const std::string& yaml_path)
        : LowerBody15PolicyBase(FSMStateName::SKILL_CAST, "SkillCast(onnx)", sc, po, yaml_path)
    {
        target_skill1_ = read_fvec(YAML::LoadFile(std::filesystem::absolute(yaml_path).string()), "upper_target_angles_skill_1");
        target_skill2_ = read_fvec(YAML::LoadFile(std::filesystem::absolute(yaml_path).string()), "upper_target_angles_skill_2");
        target_skill4_ = read_fvec(YAML::LoadFile(std::filesystem::absolute(yaml_path).string()), "upper_target_angles_skill_4");
        printf("[SkillCast] Loaded: %s\n", model_path_.c_str());
    }

    void run() override
    {
        run_lower_body(false);
        std::vector<float> target = default_upper();
        if (sc_.skill_cmd == FSMCommand::SKILL_1 && target_skill1_.size() == upper_idx_.size()) target = target_skill1_;
        else if (sc_.skill_cmd == FSMCommand::SKILL_2 && target_skill2_.size() == upper_idx_.size()) target = target_skill2_;
        else if (sc_.skill_cmd == FSMCommand::SKILL_4 && target_skill4_.size() == upper_idx_.size()) target = target_skill4_;

        advance_alpha();
        for (size_t j = 0; j < upper_idx_.size(); j++) {
            int m = upper_idx_[j];
            po_.actions[m] = upper_init_[j] * (1.f - alpha_) + target[j] * alpha_;
        }
    }

    FSMStateName check_change() override
    {
        if (cur_step_ >= num_step_ && sc_.skill_cmd == FSMCommand::SKILL_1) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::SKILL_DANCE;
        }
        if (cur_step_ >= num_step_ && sc_.skill_cmd == FSMCommand::SKILL_2) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::SKILL_KUNGFU;
        }
        if (cur_step_ >= num_step_ && sc_.skill_cmd == FSMCommand::SKILL_4) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::SKILL_KUNGFU2;
        }
        if (sc_.skill_cmd == FSMCommand::PASSIVE) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::PASSIVE;
        }
        sc_.skill_cmd = FSMCommand::INVALID;
        return FSMStateName::SKILL_COOLDOWN;
    }

private:
    std::vector<float> default_upper() const
    {
        std::vector<float> out(upper_idx_.size(), 0.f);
        for (size_t i = 0; i < upper_idx_.size(); i++) {
            int m = upper_idx_[i];
            out[i] = m < (int)default_angles_.size() ? default_angles_[m] : 0.f;
        }
        return out;
    }

    std::vector<float> target_skill1_, target_skill2_, target_skill4_;
};

} // namespace onnx_skill
