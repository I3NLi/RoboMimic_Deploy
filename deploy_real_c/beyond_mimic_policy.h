/**
 * beyond_mimic_policy.h
 * C++ port of policy/beyond_mimic/BeyondMimic.py
 *
 * Drop-in FSMState implementation that:
 *   - Loads config from BeyondMimic.yaml (via yaml-cpp)
 *   - Runs ONNX inference via onnxruntime C++ API
 *   - Applies all quaternion transforms, mj2lab remapping,
 *     observation history, and tau-limited target-position calc
 *
 * Requires:
 *   - onnxruntime C++ headers + libonnxruntime.so
 *   - yaml-cpp headers + libyaml-cpp.so
 *
 * Usage:
 *   auto policy = std::make_unique<BeyondMimicPolicy>(state_cmd, policy_output,
 *                     "/path/to/BeyondMimic.yaml");
 *   fsm.register_policy(FSMStateName::SKILL_BEYOND_MIMIC, std::move(policy));
 */

#pragma once

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <filesystem>
#include <cctype>

#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

// deploy_real.cpp declares these; include order matters — this header
// must be included AFTER the base types are defined (FSMState, StateAndCmd, etc.)
// (or include your common header that defines them first).

// ============================================================
// Tiny linear-algebra helpers (no external deps)
// ============================================================

namespace qmath {

using Vec3  = std::array<float, 3>;
using Vec4  = std::array<float, 4>;   // quaternion [w,x,y,z]
using Mat33 = std::array<float, 9>;   // row-major 3×3

inline Vec4 normalize(Vec4 q)
{
    float n = std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (n < 1e-6f) return {1,0,0,0};
    return {q[0]/n, q[1]/n, q[2]/n, q[3]/n};
}

/** Hamilton product of two [w,x,y,z] quaternions. */
inline Vec4 qmul(Vec4 a, Vec4 b)
{
    float w1=a[0],x1=a[1],y1=a[2],z1=a[3];
    float w2=b[0],x2=b[1],y2=b[2],z2=b[3];
    // same formula as Python quat_mul
    float ww = (z1+x1)*(x2+y2);
    float yy = (w1-y1)*(w2+z2);
    float zz = (w1+y1)*(w2-z2);
    float xx = ww+yy+zz;
    float qq = 0.5f*(xx+(z1-x1)*(x2-y2));
    float w  = qq - ww + (z1-y1)*(y2-z2);
    float x  = qq - xx + (x1+w1)*(x2+w2);
    float y  = qq - yy + (w1-x1)*(y2+z2);
    float z  = qq - zz + (z1+y1)*(w2-x2);
    return {w,x,y,z};
}

/** Rotation matrix from [w,x,y,z] quaternion (row-major). */
inline Mat33 mat_from_quat(Vec4 q)
{
    q = normalize(q);
    float w=q[0],x=q[1],y=q[2],z=q[3];
    return {
        1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w),
          2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w),
          2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)
    };
}

/** Transpose of a row-major 3×3 matrix. */
inline Mat33 mat_T(const Mat33& m)
{
    return { m[0],m[3],m[6], m[1],m[4],m[7], m[2],m[5],m[8] };
}

/** Matrix multiplication A @ B (both row-major 3×3). */
inline Mat33 matmul(const Mat33& A, const Mat33& B)
{
    Mat33 C{};
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            for (int k=0;k<3;k++)
                C[i*3+j] += A[i*3+k]*B[k*3+j];
    return C;
}

/** Yaw-only quaternion from full quaternion. */
inline Vec4 yaw_quat(Vec4 q)
{
    float w=q[0],x=q[1],y=q[2],z=q[3];
    float yaw = std::atan2(2.f*(w*z+x*y), 1.f-2.f*(y*y+z*z));
    float h = yaw*0.5f;
    return {std::cos(h), 0.f, 0.f, std::sin(h)};
}

/** Quaternion from single-axis angle: axis 'x'/'y'/'z'. */
inline Vec4 axis_angle_quat(float angle, char axis)
{
    float h = angle*0.5f;
    float c = std::cos(h), s = std::sin(h);
    if (axis=='x') return {c,s,0,0};
    if (axis=='y') return {c,0,s,0};
    return {c,0,0,s};  // 'z'
}

/** Apply inverse of quaternion rotation to vector: q^-1 * v. */
inline Vec3 quat_apply_inverse(Vec4 q, Vec3 v)
{
    Vec4 qc = {q[0],-q[1],-q[2],-q[3]};
    Vec4 vq = {0.f, v[0], v[1], v[2]};
    Vec4 r  = qmul(qmul(qc, vq), q);
    return {r[1],r[2],r[3]};
}

/** Clamp float to [-lim, lim]. */
inline float fclamp(float v, float lim)
{
    return v < -lim ? -lim : (v > lim ? lim : v);
}

inline float fclamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

inline bool fisfinite(float v) { return std::isfinite(v); }

} // namespace qmath

// ============================================================
// YAML config for BeyondMimic
// ============================================================

struct BeyondMimicConfig {
    std::string onnx_path;
    int         motion_length   { 0 };
    float       switch_to_loco_delay_s { 0.0f };
    bool        use_torso_quat_correction { true };
    int         num_actions     { 29 };
    int         num_obs         { 154 };
    float       tau_limit_scale { 1.0f };
    float       obs_clip        { 100.0f };
    float       action_clip     { 20.0f };
    float       q_clip          { 6.5f };
    float       dq_clip         { 80.0f };
    float       ang_vel_clip    { 80.0f };

    std::vector<float> tau_limit;
    std::vector<float> kp_lab;
    std::vector<float> kd_lab;
    std::vector<float> default_angles_lab;
    std::vector<float> action_scale_lab;
    std::vector<int>   mj2lab;

    static BeyondMimicConfig load(const std::string& yaml_path)
    {
        YAML::Node node = YAML::LoadFile(yaml_path);
        BeyondMimicConfig c;
        namespace fs = std::filesystem;
        const fs::path yaml_abs = fs::absolute(fs::path(yaml_path));
        const fs::path yaml_dir = yaml_abs.parent_path();

        if (node["onnx_path"]) {
            std::string raw_onnx = node["onnx_path"].as<std::string>();
            fs::path onnx_path(raw_onnx);
            if (onnx_path.is_absolute()) {
                c.onnx_path = onnx_path.string();
            } else {
                // Mirror Python BeyondMimic.py behavior:
                //   config/BeyondMimic.yaml + "lafan/xxx.onnx"
                //   -> ../model/lafan/xxx.onnx
                const fs::path candidate_model = yaml_dir.parent_path() / "model" / onnx_path;
                const fs::path candidate_same_dir = yaml_dir / onnx_path;
                if (fs::exists(candidate_model)) {
                    c.onnx_path = candidate_model.string();
                } else if (fs::exists(candidate_same_dir)) {
                    c.onnx_path = candidate_same_dir.string();
                } else {
                    // Keep deterministic fallback for readable error reporting.
                    c.onnx_path = candidate_model.string();
                }
            }
        }
        if (node["motion_length"])
            c.motion_length = node["motion_length"].as<int>();
        if (node["switch_to_loco_delay_s"])
            c.switch_to_loco_delay_s = node["switch_to_loco_delay_s"].as<float>();
        if (node["use_torso_quat_correction"]) {
            const YAML::Node v = node["use_torso_quat_correction"];
            try {
                c.use_torso_quat_correction = v.as<bool>();
            } catch (...) {
                try {
                    std::string s = v.as<std::string>();
                    std::transform(s.begin(), s.end(), s.begin(),
                                   [](unsigned char ch){ return (char)std::tolower(ch); });
                    c.use_torso_quat_correction =
                        (s == "1" || s == "true" || s == "yes" || s == "on");
                } catch (...) {}
            }
        }
        if (node["num_actions"])
            c.num_actions = node["num_actions"].as<int>();
        if (node["num_obs"])
            c.num_obs = node["num_obs"].as<int>();
        if (node["tau_limit_scale"])
            c.tau_limit_scale = node["tau_limit_scale"].as<float>();
        if (node["obs_clip"])      c.obs_clip     = node["obs_clip"].as<float>();
        if (node["action_clip"])   c.action_clip  = node["action_clip"].as<float>();
        if (node["q_clip"])        c.q_clip       = node["q_clip"].as<float>();
        if (node["dq_clip"])       c.dq_clip      = node["dq_clip"].as<float>();
        if (node["ang_vel_clip"])  c.ang_vel_clip = node["ang_vel_clip"].as<float>();

        auto read_fvec = [&](const char* key, std::vector<float>& v) {
            if (node[key])
                for (const auto& e : node[key]) v.push_back(e.as<float>());
        };
        auto read_ivec = [&](const char* key, std::vector<int>& v) {
            if (node[key])
                for (const auto& e : node[key]) v.push_back(e.as<int>());
        };

        read_fvec("tau_limit",          c.tau_limit);
        read_fvec("kp_lab",             c.kp_lab);
        read_fvec("kd_lab",             c.kd_lab);
        read_fvec("default_angles_lab", c.default_angles_lab);
        read_fvec("action_scale_lab",   c.action_scale_lab);
        read_ivec("mj2lab",             c.mj2lab);
        return c;
    }
};

// ============================================================
// BeyondMimicPolicy — FSMState subclass
// ============================================================

class BeyondMimicPolicy : public FSMState {
public:
    /**
     * @param sc          Robot state + command struct (shared with FSM)
     * @param po          Policy output struct (shared with FSM)
     * @param yaml_path   Absolute path to BeyondMimic.yaml
     */
    BeyondMimicPolicy(StateAndCmd& sc, PolicyOutput& po,
                      const std::string& yaml_path,
                      float control_dt = 0.02f)
        : FSMState(FSMStateName::SKILL_BEYOND_MIMIC, "BeyondMimic", sc, po),
          ort_env_(ORT_LOGGING_LEVEL_WARNING, "BeyondMimic")
    {
        switch_to_loco_dt_ = std::max(1e-6f, control_dt);
        // ── load config ──────────────────────────────────────────
        cfg_ = BeyondMimicConfig::load(yaml_path);
        na_  = cfg_.num_actions;
        no_  = cfg_.num_obs;

        // ── build per-joint arrays in MJ order ───────────────────
        kps_mj_.assign(na_, 0.f);
        kds_mj_.assign(na_, 0.f);
        tau_limit_mj_.assign(na_, 0.f);

        for (int i = 0; i < (int)cfg_.mj2lab.size(); i++) {
            int lab_i = cfg_.mj2lab[i];
            if (lab_i >= 0 && lab_i < (int)cfg_.kp_lab.size())
                kps_mj_[lab_i] = cfg_.kp_lab[i];
            if (lab_i >= 0 && lab_i < (int)cfg_.kd_lab.size())
                kds_mj_[lab_i] = cfg_.kd_lab[i];
            if (lab_i >= 0 && lab_i < (int)cfg_.tau_limit.size())
                tau_limit_mj_[lab_i] = cfg_.tau_limit[i];
        }

        // ── load ONNX model ──────────────────────────────────────
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(1);
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_ = std::make_unique<Ort::Session>(
            ort_env_, cfg_.onnx_path.c_str(), opts);

        Ort::AllocatorWithDefaultOptions alloc;
        n_inputs_  = session_->GetInputCount();
        n_outputs_ = session_->GetOutputCount();

        input_names_owned_.reserve(n_inputs_);
        output_names_owned_.reserve(n_outputs_);
        for (size_t i = 0; i < n_inputs_; i++) {
            auto s = session_->GetInputNameAllocated(i, alloc);
            input_names_owned_.push_back(s ? std::string(s.get()) : std::string());
        }
        for (size_t i = 0; i < n_outputs_; i++) {
            auto s = session_->GetOutputNameAllocated(i, alloc);
            output_names_owned_.push_back(s ? std::string(s.get()) : std::string());
        }
        input_names_.clear();
        output_names_.clear();
        input_names_.reserve(input_names_owned_.size());
        output_names_.reserve(output_names_owned_.size());
        for (auto& name : input_names_owned_) {
            input_names_.push_back(name.c_str());
        }
        for (auto& name : output_names_owned_) {
            output_names_.push_back(name.c_str());
        }

        // ── read ONNX metadata ───────────────────────────────────
        parse_metadata();
        if (cfg_.motion_length >= 0 && model_motion_length_ > 0) {
            cfg_.motion_length = model_motion_length_;
            printf("[BeyondMimic] using model motion_length=%d\n", cfg_.motion_length);
        }

        // ── allocate buffers ─────────────────────────────────────
        action_buf_.assign(na_, 0.f);
        ref_joint_pos_.assign(na_, 0.f);
        ref_joint_vel_.assign(na_, 0.f);
        ref_body_quat_w_.assign(14*4, 0.f);  // [14, 4] flattened
        ref_body_quat_w_[0] = 1.f;            // identity for body 0

        obs_buf_.assign(no_, 0.f);
        obs_history_.assign(history_length_ * per_frame_dim_, 0.f);

        printf("[BeyondMimic] Loaded: %s\n", cfg_.onnx_path.c_str());
        printf("              na=%d  no=%d  history=%d  anchor_body=%d\n",
               na_, no_, history_length_, anchor_body_idx_);
    }

    // ── FSMState interface ────────────────────────────────────────

    void enter() override
    {
        counter_step_ = 0;
        paused_ref_valid_ = false;
        std::fill(obs_history_.begin(), obs_history_.end(), 0.f);
        std::fill(action_buf_.begin(), action_buf_.end(), 0.f);
        std::fill(ref_joint_pos_.begin(), ref_joint_pos_.end(), 0.f);
        std::fill(ref_joint_vel_.begin(), ref_joint_vel_.end(), 0.f);
        std::fill(ref_body_quat_w_.begin(), ref_body_quat_w_.end(), 0.f);
        ref_body_quat_w_[0] = 1.f;

        // warm-up run with zeros so outputs are valid before first real step
        warm_up();
        printf("[BeyondMimic] Entered\n");
    }

    void run() override
    {
        bool paused = sc_.pause;
        if (paused && !paused_ref_valid_) {
            paused_ref_joint_pos_  = ref_joint_pos_;
            paused_ref_joint_vel_  = ref_joint_vel_;
            paused_ref_body_quat_w_ = ref_body_quat_w_;
            paused_ref_valid_ = true;
        }
        if (!paused && paused_ref_valid_) {
            paused_ref_valid_ = false;
        }
        if (paused) {
            ref_joint_pos_    = paused_ref_joint_pos_;
            ref_joint_vel_    = paused_ref_joint_vel_;
            ref_body_quat_w_  = paused_ref_body_quat_w_;
        }

        // ── read + sanitize state ─────────────────────────────────
        qmath::Vec4 robot_quat = sanitize_quat(sc_.base_quat);
        std::vector<float> qj_mj = sanitize_vec(
            std::vector<float>(sc_.q.begin(), sc_.q.begin()+na_), cfg_.q_clip);
        std::vector<float> dqj_mj = sanitize_vec(
            std::vector<float>(sc_.dq.begin(), sc_.dq.begin()+na_), cfg_.dq_clip);

        // ── lab-frame joint positions ─────────────────────────────
        std::vector<float> qj(na_, 0.f);   // lab order, relative to default
        for (int i = 0; i < na_; i++) {
            int lab_i = mj_index(i);
            float raw = (lab_i >= 0 && lab_i < na_) ? qj_mj[lab_i] : 0.f;
            float def = (i < (int)cfg_.default_angles_lab.size()) ? cfg_.default_angles_lab[i] : 0.f;
            qj[i] = std::clamp(raw - def, -cfg_.q_clip, cfg_.q_clip);
        }

        using namespace qmath;
        if (cfg_.use_torso_quat_correction) {
            // beyond mimic uses torso as anchor; correct pelvis quat by waist joints
            // the Python indexing is qj[2]=yaw, qj[5]=roll, qj[8]=pitch in LAB order
            float torso_yaw   = (na_>2) ? qj[2] : 0.f;
            float torso_roll  = (na_>5) ? qj[5] : 0.f;
            float torso_pitch = (na_>8) ? qj[8] : 0.f;
            Vec4 qy = axis_angle_quat(torso_yaw,   'z');
            Vec4 qr = axis_angle_quat(torso_roll,  'x');
            Vec4 qp = axis_angle_quat(torso_pitch, 'y');
            Vec4 delta = qmul(qy, qmul(qr, qp));
            robot_quat = normalize(qmul(robot_quat, delta));
        }

        // ── anchor orientation in world frame ─────────────────────
        Vec4 ref_anchor_w = sanitize_quat(get_ref_anchor_quat());

        // ── on first 2 steps, compute init→world rotation ─────────
        if (counter_step_ < 2) {
            Mat33 init_to_anchor = mat_from_quat(yaw_quat(ref_anchor_w));
            Mat33 world_to_anchor = mat_from_quat(yaw_quat(robot_quat));
            init_to_world_ = matmul(world_to_anchor, mat_T(init_to_anchor));
            counter_step_++;
            // Match Python: return early without overriding policy_output.
            return;
        }

        // ── motion_anchor_ori_b: R_robot^T @ init_to_world @ R_anchor ──
        Mat33 R_robot  = mat_from_quat(robot_quat);
        Mat33 R_anchor = mat_from_quat(ref_anchor_w);
        Mat33 motion_anchor_ori_b = matmul(mat_T(R_robot),
                                     matmul(init_to_world_, R_anchor));

        // ── build observation ─────────────────────────────────────
        std::vector<float> ang_vel = sanitize_vec(
            std::vector<float>(sc_.ang_vel.begin(), sc_.ang_vel.end()),
            cfg_.ang_vel_clip);

        Vec3 base_gravity_v = sc_.gravity_ori;
        bool gravity_valid =
            std::isfinite(base_gravity_v[0]) &&
            std::isfinite(base_gravity_v[1]) &&
            std::isfinite(base_gravity_v[2]);
        std::vector<float> base_gravity = gravity_valid
            ? sanitize_vec({base_gravity_v[0], base_gravity_v[1], base_gravity_v[2]}, 2.0f)
            : sanitize_vec(gravity_from_quat(robot_quat), 2.0f);

        std::vector<float> dqj(na_, 0.f);
        for (int i = 0; i < na_; i++) {
            int lab_i = mj_index(i);
            dqj[i] = (lab_i >= 0 && lab_i < na_) ? dqj_mj[lab_i] : 0.f;
        }

        std::vector<float> frame_obs = build_frame_obs(
            motion_anchor_ori_b, base_gravity, ang_vel, qj, dqj);
        frame_obs = sanitize_vec(frame_obs, cfg_.obs_clip);
        pad_to(frame_obs, (size_t)per_frame_dim_);

        std::vector<float> obs_input = append_history(frame_obs);
        pad_to(obs_input, (size_t)no_);
        obs_input = sanitize_vec(obs_input, cfg_.obs_clip);

        // ── ONNX inference ────────────────────────────────────────
        auto mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        std::array<int64_t,2> obs_shape  {1, (int64_t)no_};
        std::array<int64_t,2> step_shape {1, 1};
        float step_val = (float)counter_step_;
        if (cfg_.motion_length < 0) {
            int loop_steps = get_loop_steps();
            step_val = (float)(counter_step_ % std::max(1, loop_steps));
        }
        if (sc_.policy_step_override >= 0)
            step_val = (float)sc_.policy_step_override;

        std::vector<Ort::Value> inputs;
        inputs.push_back(Ort::Value::CreateTensor<float>(
            mem, obs_input.data(), obs_input.size(),
            obs_shape.data(), obs_shape.size()));
        inputs.push_back(Ort::Value::CreateTensor<float>(
            mem, &step_val, 1,
            step_shape.data(), step_shape.size()));

        std::vector<Ort::Value> outputs;
        try {
            outputs = session_->Run(
                Ort::RunOptions{nullptr},
                input_names_.data(), inputs.data(), inputs.size(),
                output_names_.data(), n_outputs_);
        } catch (const std::exception& e) {
            printf("[BeyondMimic][WARN] ONNX inference failed: %s\n", e.what());
            fill_output_from_buf(qj_mj, dqj_mj);
            if (!paused) counter_step_++;
            return;
        }

        // ── parse outputs ─────────────────────────────────────────
        // output[0]: action [1, na]
        // output[1]: ref_joint_pos [1, na]
        // output[2]: ref_joint_vel [1, na]
        // output[4]: ref_body_quat_w [1, 14, 4]
        copy_tensor(outputs[0], action_buf_,       na_);
        if (!paused) {
            copy_tensor(outputs[1], ref_joint_pos_, na_);
            copy_tensor(outputs[2], ref_joint_vel_, na_);
            if (n_outputs_ > 4)
                copy_tensor(outputs[4], ref_body_quat_w_, 14*4);
        }

        // ── decode target joint positions ─────────────────────────
        // target_lab = action * action_scale + default_angles  (lab order)
        // target_mj[mj2lab] = target_lab
        std::vector<float> action_vec = sanitize_vec(action_buf_, cfg_.action_clip);
        std::vector<float> target_lab(na_, 0.f);
        for (int i = 0; i < na_; i++) {
            float sc = (i < (int)cfg_.action_scale_lab.size()) ? cfg_.action_scale_lab[i] : 1.f;
            float def= (i < (int)cfg_.default_angles_lab.size())? cfg_.default_angles_lab[i]: 0.f;
            target_lab[i] = std::clamp(action_vec[i]*sc + def, -cfg_.q_clip, cfg_.q_clip);
        }

        std::vector<float> target_mj(na_, 0.f);
        for (int i = 0; i < na_; i++) {
            int lab_i = mj_index(i);
            if (lab_i >= 0 && lab_i < na_) target_mj[lab_i] = target_lab[i];
        }
        target_mj = sanitize_vec(target_mj, cfg_.q_clip);

        // ── tau-limit the target position ────────────────────────
        // τ = (target - q) * kp + (0 - dq) * kd   clipped to tau_limit
        // target_limited = q + (τ + kd*dq) / kp
        for (int i = 0; i < na_; i++) {
            float kp = std::max(kps_mj_[i], 0.f);
            float kd = std::max(kds_mj_[i], 0.f);
            float tl = std::max(tau_limit_mj_[i] * cfg_.tau_limit_scale, 0.f);
            float tau = (target_mj[i] - qj_mj[i]) * kp + (0.f - dqj_mj[i]) * kd;
            tau = std::isfinite(tau) ? std::clamp(tau, -tl, tl) : 0.f;
            if (kp > 1e-6f)
                target_mj[i] = qj_mj[i] + (tau + kd * dqj_mj[i]) / kp;
        }
        target_mj = sanitize_vec(target_mj, cfg_.q_clip);

        // ── write policy output ───────────────────────────────────
        for (int i = 0; i < na_ && i < G1_NUM_MOTOR; i++) {
            po_.actions[i] = target_mj[i];
            po_.kps[i]     = kps_mj_[i];
            po_.kds[i]     = kds_mj_[i];
        }

        if (!paused) counter_step_++;
    }

    void exit() override
    {
        counter_step_ = 0;
        action_complete_hold_steps_ = 0;
        paused_ref_valid_ = false;
        std::fill(action_buf_.begin(),   action_buf_.end(),   0.f);
        std::fill(obs_history_.begin(),  obs_history_.end(),  0.f);
        std::fill(ref_joint_pos_.begin(),ref_joint_pos_.end(),0.f);
        std::fill(ref_joint_vel_.begin(),ref_joint_vel_.end(),0.f);
        printf("[BeyondMimic] Exited\n");
    }

    FSMStateName check_change() override
    {
        if (is_action_complete()) {
            // switch_to_loco_delay_s semantics:
            // <0: never auto-return; 0: return immediately; >0: return after delay.
            if (cfg_.switch_to_loco_delay_s < 0.0f) {
                sc_.skill_cmd = FSMCommand::INVALID;
                return FSMStateName::SKILL_BEYOND_MIMIC;
            }
            int delay_steps = std::max(
                0,
                (int)std::round(cfg_.switch_to_loco_delay_s /
                                std::max(1e-6f, switch_to_loco_dt_)));
            action_complete_hold_steps_++;
            if (action_complete_hold_steps_ >= delay_steps) {
                sc_.skill_cmd = FSMCommand::INVALID;
                return FSMStateName::SKILL_COOLDOWN;
            }
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::SKILL_BEYOND_MIMIC;
        }
        action_complete_hold_steps_ = 0;

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
        return FSMStateName::SKILL_BEYOND_MIMIC;
    }

private:
    // ── ONNX ─────────────────────────────────────────────────────
    Ort::Env                         ort_env_;
    std::unique_ptr<Ort::Session>    session_;
    size_t                           n_inputs_  { 0 };
    size_t                           n_outputs_ { 0 };
    std::vector<std::string>         input_names_owned_;
    std::vector<std::string>         output_names_owned_;
    std::vector<const char*>         input_names_;
    std::vector<const char*>         output_names_;

    // ── config + derived arrays ───────────────────────────────────
    BeyondMimicConfig                cfg_;
    int                              na_ { 29 };
    int                              no_ { 154 };
    std::vector<float>               kps_mj_;
    std::vector<float>               kds_mj_;
    std::vector<float>               tau_limit_mj_;

    // ── model metadata ────────────────────────────────────────────
    std::vector<std::string>         obs_names_;
    int                              per_frame_dim_ { 0 };
    int                              history_length_{ 1 };
    int                              anchor_body_idx_{ 7 };
    int                              model_motion_length_{ 0 };

    // ── runtime buffers ───────────────────────────────────────────
    std::vector<float> action_buf_;
    std::vector<float> ref_joint_pos_;
    std::vector<float> ref_joint_vel_;
    std::vector<float> ref_body_quat_w_;  // [14, 4] flat
    std::vector<float> obs_buf_;
    std::vector<float> obs_history_;      // [history_length, per_frame_dim] flat

    qmath::Mat33 init_to_world_ {
        1,0,0, 0,1,0, 0,0,1
    };
    int  counter_step_   { 0 };
    int  action_complete_hold_steps_ { 0 };
    bool loop_step_warned_ { false };
    float switch_to_loco_dt_ { 0.02f };
    bool paused_ref_valid_{ false };
    std::vector<float> paused_ref_joint_pos_;
    std::vector<float> paused_ref_joint_vel_;
    std::vector<float> paused_ref_body_quat_w_;

    // ── helpers ───────────────────────────────────────────────────

    /** Parse model metadata to set obs_names, per_frame_dim, history_length. */
    void parse_metadata()
    {
        // Default obs_names matching Python fallback
        obs_names_ = {
            "command",
            "motion_anchor_ori_b",
            "base_ang_vel",
            "joint_pos",
            "joint_vel",
            "actions"
        };

        Ort::ModelMetadata meta = session_->GetModelMetadata();
        Ort::AllocatorWithDefaultOptions alloc;

        auto get_meta = [&](const char* key) -> std::string {
            try {
                auto v = meta.LookupCustomMetadataMapAllocated(key, alloc);
                return v ? std::string(v.get()) : "";
            } catch (...) { return ""; }
        };

        // obs names
        std::string obs_names_str = get_meta("observation_names");
        if (!obs_names_str.empty()) {
            obs_names_ = split_csv(obs_names_str);
        }

        // model motion length (metadata takes priority when available)
        for (const char* key : {"motion_length", "motion_len", "traj_length"}) {
            std::string raw = get_meta(key);
            if (raw.empty()) continue;
            try {
                int v = (int)std::round(std::stof(raw));
                if (v > 0) {
                    model_motion_length_ = v;
                    printf("[BeyondMimic] motion_length from metadata: %d\n", model_motion_length_);
                    break;
                }
            } catch (...) {}
        }

        // anchor body index
        std::string body_names_str = get_meta("body_names");
        std::string anchor_name    = get_meta("anchor_body_name");
        if (!body_names_str.empty() && !anchor_name.empty()) {
            auto bnames = split_csv(body_names_str);
            for (int i = 0; i < (int)bnames.size(); i++) {
                if (bnames[i] == anchor_name) { anchor_body_idx_ = i; break; }
            }
        }

        // per_frame_dim from obs_names
        per_frame_dim_ = compute_per_frame_dim(obs_names_);

        // history length
        std::string hist_str = get_meta("observation_history_lengths");
        if (!hist_str.empty()) {
            auto vals = parse_csv_ints(hist_str);
            if (!vals.empty()) {
                bool all_eq = std::all_of(vals.begin(), vals.end(),
                                          [&](int x){ return x == vals[0]; });
                if (all_eq) history_length_ = vals[0];
            }
        }
        // infer from model input shape
        if (history_length_ <= 1 && per_frame_dim_ > 0) {
            auto ti = session_->GetInputTypeInfo(0);
            auto shape = ti.GetTensorTypeAndShapeInfo().GetShape();
            if (!shape.empty()) {
                int64_t total = shape.back();
                if (total > 0 && total % per_frame_dim_ == 0)
                    history_length_ = (int)(total / per_frame_dim_);
            }
        }
        history_length_ = std::max(1, history_length_);

        // override num_obs from actual model input dimension
        {
            auto ti = session_->GetInputTypeInfo(0);
            auto shape = ti.GetTensorTypeAndShapeInfo().GetShape();
            if (!shape.empty() && shape.back() > 0)
                no_ = (int)shape.back();
        }

        // action_scale from metadata
        std::string as_str = get_meta("action_scale");
        if (!as_str.empty()) {
            auto asc = parse_csv_floats(as_str);
            if ((int)asc.size() == na_) cfg_.action_scale_lab = asc;
        }

        // default_joint_pos from metadata
        std::string dj_str = get_meta("default_joint_pos");
        if (!dj_str.empty()) {
            auto djp = parse_csv_floats(dj_str);
            if ((int)djp.size() == na_) cfg_.default_angles_lab = djp;
        }

        obs_history_.assign(history_length_ * per_frame_dim_, 0.f);
    }

    int compute_per_frame_dim(const std::vector<std::string>& names) const
    {
        int dim = 0;
        for (auto& n : names) {
            if (n == "command")               dim += na_ * 2;
            else if (n == "motion_anchor_ori_b") dim += 6;
            else if (n == "motion_anchor_pos_b") dim += 3;
            else if (n == "base_gravity" ||
                     n == "base_lin_vel"  ||
                     n == "base_ang_vel")      dim += 3;
            else if (n == "joint_pos" ||
                     n == "joint_vel"  ||
                     n == "actions")           dim += na_;
        }
        return dim;
    }

    /** Extract ref anchor quaternion [w,x,y,z] from ref_body_quat_w_. */
    qmath::Vec4 get_ref_anchor_quat() const
    {
        int idx = std::min(anchor_body_idx_, 13);
        int off = idx * 4;
        if (off + 3 >= (int)ref_body_quat_w_.size())
            return {1,0,0,0};
        return { ref_body_quat_w_[off+0], ref_body_quat_w_[off+1],
                 ref_body_quat_w_[off+2], ref_body_quat_w_[off+3] };
    }

    /** Build single-frame observation vector. */
    std::vector<float> build_frame_obs(
        const qmath::Mat33& motion_anchor_ori_b,
        const std::vector<float>& base_gravity,
        const std::vector<float>& ang_vel,
        const std::vector<float>& qj,
        const std::vector<float>& dqj) const
    {
        std::vector<float> parts;
        parts.reserve(per_frame_dim_);

        for (auto& name : obs_names_) {
            if (name == "command") {
                for (float v : ref_joint_pos_) parts.push_back(v);
                for (float v : ref_joint_vel_) parts.push_back(v);
            } else if (name == "motion_anchor_ori_b") {
                // Match Python: motion_anchor_ori_b[:, :2].reshape(-1)
                // i.e. first two columns of 3x3, row-major flatten.
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 2; c++)
                        parts.push_back(motion_anchor_ori_b[r*3+c]);
            } else if (name == "motion_anchor_pos_b") {
                parts.push_back(0.f); parts.push_back(0.f); parts.push_back(0.f);
            } else if (name == "base_gravity") {
                for (int i=0;i<3;i++) parts.push_back(i<(int)base_gravity.size()?base_gravity[i]:0.f);
            } else if (name == "base_lin_vel") {
                parts.push_back(0.f); parts.push_back(0.f); parts.push_back(0.f);
            } else if (name == "base_ang_vel") {
                for (int i=0;i<3;i++) parts.push_back(i<(int)ang_vel.size()?ang_vel[i]:0.f);
            } else if (name == "joint_pos") {
                for (float v : qj) parts.push_back(v);
            } else if (name == "joint_vel") {
                for (float v : dqj) parts.push_back(v);
            } else if (name == "actions") {
                for (float v : action_buf_) parts.push_back(v);
            }
        }
        return parts;
    }

    /** Roll history and append new frame, return flattened obs. */
    std::vector<float> append_history(const std::vector<float>& frame)
    {
        if (history_length_ <= 1) return frame;

        // roll: drop oldest frame (front), shift everything, append at end
        int fd = per_frame_dim_;
        // obs_history_ is [history_length_, fd] row-major
        // shift rows [1..H-1] to [0..H-2]
        std::memmove(obs_history_.data(),
                     obs_history_.data() + fd,
                     (size_t)(history_length_-1) * fd * sizeof(float));
        // write new frame at the last row
        for (int i = 0; i < fd && i < (int)frame.size(); i++)
            obs_history_[(history_length_-1)*fd + i] = frame[i];
        return obs_history_;
    }

    /** Copy first n floats from ORT tensor into dst. */
    static void copy_tensor(const Ort::Value& t, std::vector<float>& dst, int n)
    {
        const float* ptr = t.GetTensorData<float>();
        for (int i = 0; i < n && i < (int)dst.size(); i++)
            dst[i] = std::isfinite(ptr[i]) ? ptr[i] : 0.f;
    }

    /** Fill policy output from existing buffers (used during warmup / error). */
    void fill_output_from_buf(const std::vector<float>& qj_mj,
                              const std::vector<float>& /*dqj_mj*/)
    {
        for (int i = 0; i < na_ && i < G1_NUM_MOTOR; i++) {
            po_.actions[i] = qj_mj[i];  // hold current position
            po_.kps[i]     = kps_mj_[i];
            po_.kds[i]     = kds_mj_[i];
        }
    }

    /** Warm-up: run inference once with zeros so all outputs are populated. */
    void warm_up()
    {
        auto mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        std::vector<float> zero_obs(no_, 0.f);
        float step_val = 0.f;
        std::array<int64_t,2> obs_shape  {1, (int64_t)no_};
        std::array<int64_t,2> step_shape {1, 1};

        std::vector<Ort::Value> inputs;
        inputs.push_back(Ort::Value::CreateTensor<float>(
            mem, zero_obs.data(), zero_obs.size(),
            obs_shape.data(), obs_shape.size()));
        inputs.push_back(Ort::Value::CreateTensor<float>(
            mem, &step_val, 1,
            step_shape.data(), step_shape.size()));

        try {
            auto out = session_->Run(
                Ort::RunOptions{nullptr},
                input_names_.data(), inputs.data(), inputs.size(),
                output_names_.data(), n_outputs_);
            copy_tensor(out[0], action_buf_,    na_);
            copy_tensor(out[1], ref_joint_pos_, na_);
            copy_tensor(out[2], ref_joint_vel_, na_);
            if (n_outputs_ > 4)
                copy_tensor(out[4], ref_body_quat_w_, 14*4);
        } catch (...) {}
    }

    // ── misc helpers ─────────────────────────────────────────────

    qmath::Vec4 sanitize_quat(const std::array<float,4>& q) const
    {
        float n = 0.f;
        for (float v : q) n += v*v;
        n = std::sqrt(n);
        if (!std::isfinite(n) || n < 1e-6f) return {1,0,0,0};
        return { q[0]/n, q[1]/n, q[2]/n, q[3]/n };
    }

    static std::vector<float> sanitize_vec(std::vector<float> v, float clip)
    {
        for (float& x : v) {
            if (!std::isfinite(x)) x = 0.f;
            if (clip > 0.f) x = std::clamp(x, -clip, clip);
        }
        return v;
    }

    static void pad_to(std::vector<float>& v, size_t n)
    {
        if (v.size() < n) v.resize(n, 0.f);
        else if (v.size() > n) v.resize(n);
    }

    static std::vector<std::string> split_csv(const std::string& s)
    {
        std::vector<std::string> out;
        size_t i = 0;
        while (i < s.size()) {
            size_t j = s.find(',', i);
            if (j == std::string::npos) j = s.size();
            std::string tok = s.substr(i, j-i);
            // trim whitespace
            size_t b = tok.find_first_not_of(" \t\r\n");
            if (b != std::string::npos) {
                size_t e = tok.find_last_not_of(" \t\r\n");
                out.push_back(tok.substr(b, e-b+1));
            }
            i = j+1;
        }
        return out;
    }

    static std::vector<int> parse_csv_ints(const std::string& s)
    {
        std::vector<int> out;
        for (auto& tok : split_csv(s)) {
            try { out.push_back(std::stoi(tok)); } catch (...) {}
        }
        return out;
    }

    static std::vector<float> parse_csv_floats(const std::string& s)
    {
        std::vector<float> out;
        for (auto& tok : split_csv(s)) {
            try { out.push_back(std::stof(tok)); } catch (...) {}
        }
        return out;
    }

    int mj_index(int lab_idx) const
    {
        if (lab_idx < 0) return -1;
        if (lab_idx >= (int)cfg_.mj2lab.size()) return lab_idx;
        return cfg_.mj2lab[lab_idx];
    }

    std::vector<float> gravity_from_quat(const qmath::Vec4& q) const
    {
        qmath::Vec3 g = qmath::quat_apply_inverse(q, qmath::Vec3{0.f, 0.f, -1.f});
        return {g[0], g[1], g[2]};
    }

    bool is_action_complete() const
    {
        if (cfg_.motion_length <= 0) return false;
        return counter_step_ >= cfg_.motion_length;
    }

    int get_loop_steps()
    {
        if (model_motion_length_ > 1) {
            return model_motion_length_;
        }
        int raw_steps = std::abs(cfg_.motion_length);
        if (raw_steps <= 1) {
            int fallback_steps = std::max(
                1,
                (int)std::round(10.0f / std::max(1e-6f, switch_to_loco_dt_)));
            if (!loop_step_warned_) {
                printf("[BeyondMimic][WARN] motion_length=%d causes 1-step loop; "
                       "fallback loop_steps=%d\n",
                       cfg_.motion_length, fallback_steps);
                loop_step_warned_ = true;
            }
            return fallback_steps;
        }
        return raw_steps;
    }
};
