/**
 * robot_controller.cpp
 * C++ controller runtime for the MagicBot Z1 robot.
 *
 * Mirrors the Python version faithfully:
 *  - DDS communication via unitree_sdk2 C++ API
 *  - RemoteController parsing (same byte layout as Python struct.unpack)
 *  - Gravity orientation from IMU quaternion
 *  - SafetyFilter: action/gain clamping, fault handling
 *  - HoldToConfirm: N-frame button hold to confirm commands
 *  - FSM: PassiveMode, FixedPose built-in; plugin interface for ONNX policies
 *  - 50 Hz control loop with overtime detection
 *
 * Usage:
 *   ./robot_controller_onnx [network_interface] [num_joints] [--yaml PATH] [--track-yaml PATH]
 *   e.g.  ./robot_controller_onnx enp4s0 24
 *
 * Build: see CMakeLists.txt
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <cfloat>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>
#include <time.h>
#include <cstdint>
#include <unistd.h>
#include <signal.h>
#include <filesystem>

// unitree_sdk2 C++ headers
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

// ============================================================
// Constants
// ============================================================

static constexpr int    Z1_NUM_MOTOR    = 24;
static constexpr float  DEFAULT_CTRL_DT = 0.02f;   // 50 Hz
static constexpr int    DEFAULT_ERR_OT  = 5;
static constexpr float  DAMPING_KD      = 8.0f;

// JointZeroCheck config mirror (policies/joint_zero_check/config/JointZeroCheck.yaml)
static const std::array<float, Z1_NUM_MOTOR> JOINT_ZERO_KP = {
    113.02670959481426f,113.02670959481426f,113.02670959481426f,113.02670959481426f,59.33606165595733f,59.33606165595733f,
    113.02670959481426f,113.02670959481426f,113.02670959481426f,113.02670959481426f,59.33606165595733f,59.33606165595733f,
    113.02670959481426f,
    59.33606165595733f,59.33606165595733f,59.33606165595733f,59.33606165595733f,59.33606165595733f,
    59.33606165595733f,59.33606165595733f,59.33606165595733f,59.33606165595733f,59.33606165595733f,
    59.33606165595733f
};
static const std::array<float, Z1_NUM_MOTOR> JOINT_ZERO_KD = {
    7.1955038135764f,7.1955038135764f,7.1955038135764f,7.1955038135764f,3.7774510065684f,3.7774510065684f,
    7.1955038135764f,7.1955038135764f,7.1955038135764f,7.1955038135764f,3.7774510065684f,3.7774510065684f,
    7.1955038135764f,
    3.7774510065684f,3.7774510065684f,3.7774510065684f,3.7774510065684f,3.7774510065684f,
    3.7774510065684f,3.7774510065684f,3.7774510065684f,3.7774510065684f,3.7774510065684f,
    3.7774510065684f
};
static const std::array<float, Z1_NUM_MOTOR> JOINT_ZERO_DEFAULT = {
    0.f,0.f,0.f,0.35f,-0.18f,0.f,
    0.f,0.f,0.f,0.35f,-0.18f,0.f,
    0.f,
    0.15f,0.15f,0.f,0.5f,0.f,
    0.15f,-0.15f,0.f,0.5f,0.f,
    0.f
};

// SkillCooldown config mirror (policies/skill_cooldown/config/SkillCooldown.yaml)
static const std::array<float, Z1_NUM_MOTOR> SKILL_CD_KP = {
    113.02670959481426f,113.02670959481426f,113.02670959481426f,113.02670959481426f,59.33606165595733f,59.33606165595733f,
    113.02670959481426f,113.02670959481426f,113.02670959481426f,113.02670959481426f,59.33606165595733f,59.33606165595733f,
    113.02670959481426f,
    59.33606165595733f,59.33606165595733f,59.33606165595733f,59.33606165595733f,59.33606165595733f,
    59.33606165595733f,59.33606165595733f,59.33606165595733f,59.33606165595733f,59.33606165595733f,
    59.33606165595733f
};
static const std::array<float, Z1_NUM_MOTOR> SKILL_CD_KD = {
    7.1955038135764f,7.1955038135764f,7.1955038135764f,7.1955038135764f,3.7774510065684f,3.7774510065684f,
    7.1955038135764f,7.1955038135764f,7.1955038135764f,7.1955038135764f,3.7774510065684f,3.7774510065684f,
    7.1955038135764f,
    3.7774510065684f,3.7774510065684f,3.7774510065684f,3.7774510065684f,3.7774510065684f,
    3.7774510065684f,3.7774510065684f,3.7774510065684f,3.7774510065684f,3.7774510065684f,
    3.7774510065684f
};
static const std::array<float, Z1_NUM_MOTOR> SKILL_CD_DEFAULT = {
    0.f,0.f,0.f,0.35f,-0.18f,0.f,
    0.f,0.f,0.f,0.35f,-0.18f,0.f,
    0.f,
    0.15f,0.15f,0.f,0.5f,0.f,
    0.15f,-0.15f,0.f,0.5f,0.f,
    0.f
};
static const std::array<int, 0> SKILL_CD_LOWER_IDX = {};
static const std::array<int, Z1_NUM_MOTOR> SKILL_CD_UPPER_IDX = {
    0,1,2,3,4,5,6,7,8,9,10,11,
    12,13,14,15,16,17,18,19,20,21,22,23
};

// ============================================================
// CRC32 — same polynomial used by unitree examples
// ============================================================

static uint32_t crc32_core(const uint32_t* ptr, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    const uint32_t poly = 0x04c11db7u;
    for (uint32_t i = 0; i < len; i++) {
        uint32_t data = ptr[i];
        uint32_t xbit = 1u << 31;
        for (uint32_t b = 0; b < 32; b++) {
            if (crc & 0x80000000u) { crc <<= 1; crc ^= poly; }
            else                   { crc <<= 1; }
            if (data & xbit) crc ^= poly;
            xbit >>= 1;
        }
    }
    return crc;
}

// ============================================================
// Wall-clock time helper
// ============================================================

static inline double now_sec()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static inline void sleep_sec(double s)
{
    if (s <= 0.0) return;
    struct timespec ts;
    ts.tv_sec  = (time_t)s;
    ts.tv_nsec = (long)((s - ts.tv_sec) * 1e9);
    nanosleep(&ts, nullptr);
}

// ============================================================
// Config — mirrors real.yaml + safety.yaml defaults
// ============================================================

struct Config {
    // real.yaml
    std::string net             = "enp4s0";
    int         num_joints      = Z1_NUM_MOTOR;
    std::string lowcmd_topic    = "rt/lowcmd";
    std::string lowstate_topic  = "rt/lowstate";
    float       control_dt      = DEFAULT_CTRL_DT;
    int         error_over_time = DEFAULT_ERR_OT;

    // safety.yaml
    bool  safety_enable           = false;
    bool  safety_dry_run          = false;
    bool  safety_log_events       = true;
    bool  sync_on_lowstate        = false;  // shadow compare helper
    bool  wait_for_lowstate_on_init = true;
    int   command_hold_frames     = 3;
    float max_action_abs          = 3.5f;
    float max_action_delta        = 0.25f;
    float max_kp                  = 300.0f;
    float max_kd                  = 20.0f;
    float max_kp_delta            = 50.0f;
    float max_kd_delta            = 5.0f;
    float damping_kd              = DAMPING_KD;
    int   fault_hold_steps        = 25;
    float fault_latch_seconds     = 1.0f;
    int   max_faults_before_latch = 3;
};

// ============================================================
// KeyMap — matches Python KeyMap
// ============================================================

enum KeyMap {
    KEY_R1 = 0, KEY_L1 = 1, KEY_START  = 2, KEY_SELECT = 3,
    KEY_R2 = 4, KEY_L2 = 5, KEY_F1     = 6, KEY_F2     = 7,
    KEY_A  = 8, KEY_B  = 9, KEY_X      = 10, KEY_Y     = 11,
    KEY_UP = 12, KEY_RIGHT = 13, KEY_DOWN = 14, KEY_LEFT = 15
};

// ============================================================
// RemoteController — same byte parsing as Python struct.unpack
// ============================================================

class RemoteController {
public:
    float lx{}, ly{}, rx{}, ry{};
    int   button[16]{};
    bool  button_states[16]{};
    bool  button_pressed[16]{};
    bool  button_released[16]{};

    void set(const std::array<uint8_t, 40>& data)
    {
        // bytes [2:4] → uint16 little-endian button flags
        uint16_t keys = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
        for (int i = 0; i < 16; i++)
            button[i] = (keys >> i) & 1;

        memcpy(&lx, data.data() +  4, 4);
        memcpy(&rx, data.data() +  8, 4);
        memcpy(&ry, data.data() + 12, 4);
        memcpy(&ly, data.data() + 20, 4);

        for (int i = 0; i < 16; i++) button_released[i] = false;

        for (int i = 0; i < 16; i++) {
            bool cur = (button[i] == 1);
            if (button_states[i] && !cur) button_released[i] = true;
            button_states[i] = cur;
            button_pressed[i] = cur;
        }
    }

    bool is_pressed (int id) const { return id >= 0 && id < 16 && button_states[id]; }
    bool is_released(int id) const { return id >= 0 && id < 16 && button_released[id]; }
};

// ============================================================
// FSMCommand — matches Python FSMCommand enum
// ============================================================

enum class FSMCommand {
    INVALID   = -1,
    POS_RESET =  1,
    LOCO      =  2,
    PASSIVE   =  4,
    SKILL_1   =  5,
    SKILL_2   =  6,
    SKILL_3   =  7,
    SKILL_4   =  8,
    SKILL_5   =  9,
    SKILL_6   = 10,
    SKILL_7   = 11,
    PAUSE     = 12,
};

// ============================================================
// FSMStateName — matches Python FSMStateName enum
// ============================================================

enum class FSMStateName {
    INVALID            = -1,
    PASSIVE            =  1,
    FIXEDPOSE          =  2,
    SKILL_COOLDOWN     =  3,
    LOCOMODE           =  4,
    SKILL_CAST         =  5,
    SKILL_KUNGFU       =  6,
    SKILL_DANCE        =  7,
    SKILL_KICK         =  8,
    SKILL_KUNGFU2      =  9,
    SKILL_BEYOND_MIMIC = 10,
    JOINT_ZERO_CHECK   = 11,
    IMU_CALIB          = 12,
    SKILL_TRACK_MIMIC  = 13,
};

// ============================================================
// StateAndCmd — mirrors Python StateAndCmd
// ============================================================

struct StateAndCmd {
    int num_joints;

    std::array<float, Z1_NUM_MOTOR> q   {};
    std::array<float, Z1_NUM_MOTOR> dq  {};
    std::array<float, Z1_NUM_MOTOR> ddq {};
    std::array<float, Z1_NUM_MOTOR> tau_est {};

    std::array<float, 3> gravity_ori { 0.f, 0.f, -1.f };
    std::array<float, 3> ang_vel     {};
    std::array<float, 4> base_quat   { 1.f, 0.f, 0.f, 0.f };  // w,x,y,z
    std::array<float, 3> vel_cmd     {};
    int policy_step_override { -1 };  // compare helper; -1 means use local counter

    FSMCommand skill_cmd { FSMCommand::INVALID };
    bool pause { false };

    explicit StateAndCmd(int n) : num_joints(n) {}
};

// ============================================================
// PolicyOutput — mirrors Python PolicyOutput
// ============================================================

struct PolicyOutput {
    int num_joints;
    std::array<float, Z1_NUM_MOTOR> actions {};
    std::array<float, Z1_NUM_MOTOR> kps     {};
    std::array<float, Z1_NUM_MOTOR> kds     {};

    explicit PolicyOutput(int n) : num_joints(n) {}
};

// ============================================================
// Rotation helpers — mirrors Python get_gravity_orientation_real
// ============================================================

static std::array<float, 3> get_gravity_orientation(const std::array<float, 4>& q)
{
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    return {
         2.0f * (-qz * qx + qw * qy),
        -2.0f * ( qz * qy + qw * qx),
         1.0f - 2.0f * (qw * qw + qz * qz)
    };
}

// ============================================================
// HoldToConfirm — mirrors Python HoldToConfirm
// ============================================================

class HoldToConfirm {
public:
    int hold_frames;

    explicit HoldToConfirm(int frames) : hold_frames(std::max(1, frames)) {}

    bool trigger(const std::string& key, bool pressed)
    {
        if (!pressed) { count_[key] = 0; latched_[key] = false; return false; }
        if (latched_.count(key) && latched_[key]) return false;
        if (++count_[key] >= hold_frames) { latched_[key] = true; return true; }
        return false;
    }

private:
    std::map<std::string, int>  count_;
    std::map<std::string, bool> latched_;
};

// ============================================================
// SafetyFilter — mirrors Python SafetyFilter
// ============================================================

class SafetyFilter {
public:
    SafetyFilter(int num_joints, const Config& cfg)
        : num_joints_(num_joints), cfg_(cfg) {}

    void report_fault(const std::string& reason)
    {
        if (cfg_.safety_enable) fault(reason);
    }

    /**
     * Clamp actions/gains in-place.
     * Returns true if force-damping should be applied.
     */
    bool filter_actions(
        std::array<float, Z1_NUM_MOTOR>& actions,
        std::array<float, Z1_NUM_MOTOR>& kps,
        std::array<float, Z1_NUM_MOTOR>& kds)
    {
        step_++;
        if (!cfg_.safety_enable) return false;

        // NaN/Inf check
        for (int i = 0; i < num_joints_; i++) {
            if (!std::isfinite(actions[i]) ||
                !std::isfinite(kps[i])    ||
                !std::isfinite(kds[i]))
            {
                fault("NaN/Inf in actions or gains");
                actions = prev_action_; kps = prev_kp_; kds = prev_kd_;
                return true;
            }
        }

        // action abs clamp
        for (int i = 0; i < num_joints_; i++)
            actions[i] = std::clamp(actions[i], -cfg_.max_action_abs, cfg_.max_action_abs);

        // action delta clamp
        for (int i = 0; i < num_joints_; i++) {
            float d = std::clamp(actions[i] - prev_action_[i],
                                 -cfg_.max_action_delta, cfg_.max_action_delta);
            actions[i] = prev_action_[i] + d;
        }

        // kp clamp
        for (int i = 0; i < num_joints_; i++)
            kps[i] = std::clamp(kps[i], 0.0f, cfg_.max_kp);

        // kd clamp
        for (int i = 0; i < num_joints_; i++)
            kds[i] = std::clamp(kds[i], 0.0f, cfg_.max_kd);

        // kp delta clamp
        for (int i = 0; i < num_joints_; i++) {
            float d = std::clamp(kps[i] - prev_kp_[i], -cfg_.max_kp_delta, cfg_.max_kp_delta);
            kps[i] = prev_kp_[i] + d;
        }

        // kd delta clamp
        for (int i = 0; i < num_joints_; i++) {
            float d = std::clamp(kds[i] - prev_kd_[i], -cfg_.max_kd_delta, cfg_.max_kd_delta);
            kds[i] = prev_kd_[i] + d;
        }

        prev_action_ = actions;
        prev_kp_     = kps;
        prev_kd_     = kds;

        return should_force_damping();
    }

private:
    int          num_joints_;
    const Config& cfg_;
    int   step_            { 0 };
    int   fault_count_     { 0 };
    int   fault_until_step_{ 0 };
    double latch_until_    { 0.0 };

    std::array<float, Z1_NUM_MOTOR> prev_action_{};
    std::array<float, Z1_NUM_MOTOR> prev_kp_    {};
    std::array<float, Z1_NUM_MOTOR> prev_kd_    {};

    void fault(const std::string& reason)
    {
        fault_count_++;
        fault_until_step_ = std::max(fault_until_step_, step_ + cfg_.fault_hold_steps);
        if (fault_count_ >= cfg_.max_faults_before_latch)
            latch_until_ = now_sec() + cfg_.fault_latch_seconds;
        if (cfg_.safety_log_events)
            printf("[Safety] fault: %s (count=%d)\n", reason.c_str(), fault_count_);
    }

    bool should_force_damping() const
    {
        if (step_ < fault_until_step_) return true;
        if (now_sec() < latch_until_)  return true;
        return false;
    }
};

// ============================================================
// FSMState abstract base — mirrors Python FSMState
// ============================================================

class FSMState {
public:
    FSMStateName name;
    std::string  name_str;

    FSMState(FSMStateName n, const std::string& ns,
             StateAndCmd& sc, PolicyOutput& po)
        : name(n), name_str(ns), sc_(sc), po_(po) {}

    virtual void enter() {}
    virtual void run()   = 0;
    virtual void exit()  {}
    virtual FSMStateName check_change() { return name; }

    virtual ~FSMState() = default;

protected:
    StateAndCmd&  sc_;
    PolicyOutput& po_;
};

// ============================================================
// PassiveMode — zero torque, damping only
// ============================================================

class PassiveMode : public FSMState {
public:
    PassiveMode(StateAndCmd& sc, PolicyOutput& po, float kd = DAMPING_KD)
        : FSMState(FSMStateName::PASSIVE, "PassiveMode", sc, po), kd_(kd) {}

    void enter() override { printf("[PassiveMode] Entered\n"); }

    void run() override
    {
        for (int i = 0; i < sc_.num_joints; i++) {
            po_.actions[i] = 0.0f;
            po_.kps[i]     = 0.0f;
            po_.kds[i]     = kd_;
        }
    }

    FSMStateName check_change() override
    {
        if (sc_.skill_cmd == FSMCommand::POS_RESET) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::FIXEDPOSE;
        }
        sc_.skill_cmd = FSMCommand::INVALID;
        return FSMStateName::PASSIVE;
    }

private:
    float kd_;
};

// ============================================================
// FixedPose — smooth interpolation to zero joint angles
// ============================================================

class FixedPose : public FSMState {
public:
    FixedPose(StateAndCmd& sc, PolicyOutput& po, float dt = DEFAULT_CTRL_DT)
        : FSMState(FSMStateName::FIXEDPOSE, "FixedPose", sc, po),
          control_dt_(dt) {}

    void enter() override
    {
        init_pos_ = sc_.q;
        elapsed_  = 0.0f;
        printf("[FixedPose] Interpolating to default pose over %.1f s\n", duration_);
    }

    void run() override
    {
        elapsed_ += control_dt_;
        float ratio = std::min(elapsed_ / duration_, 1.0f);

        for (int i = 0; i < sc_.num_joints; i++) {
            po_.actions[i] = init_pos_[i] * (1.0f - ratio) + default_pos_[i] * ratio;
            po_.kps[i]     = kps_[i];
            po_.kds[i]     = kds_[i];
        }
    }

    void exit() override
    {
        for (int i = 0; i < sc_.num_joints; i++) {
            po_.actions[i] = default_pos_[i];
            po_.kps[i]     = kps_[i];
            po_.kds[i]     = kds_[i];
        }
    }

    FSMStateName check_change() override
    {
        if (sc_.skill_cmd == FSMCommand::LOCO) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::LOCOMODE;
        }
        if (sc_.skill_cmd == FSMCommand::PASSIVE) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::PASSIVE;
        }
        sc_.skill_cmd = FSMCommand::INVALID;
        return FSMStateName::FIXEDPOSE;
    }

private:
    float control_dt_;
    float duration_ { 2.0f };
    float elapsed_  { 0.0f };
    std::array<float, Z1_NUM_MOTOR> init_pos_{};
    const std::array<float, Z1_NUM_MOTOR> default_pos_ { JOINT_ZERO_DEFAULT };
    const std::array<float, Z1_NUM_MOTOR> kps_ { JOINT_ZERO_KP };
    const std::array<float, Z1_NUM_MOTOR> kds_ { JOINT_ZERO_KD };
};

// ============================================================
// LocoMode placeholder — add ONNX/libtorch policy here
// ============================================================

class LocoModePlaceholder : public FSMState {
public:
    LocoModePlaceholder(StateAndCmd& sc, PolicyOutput& po, float kd = DAMPING_KD)
        : FSMState(FSMStateName::LOCOMODE, "LocoMode(stub)", sc, po), kd_(kd) {}

    void enter() override
    {
        printf("[LocoMode] WARNING: neural network not loaded — damping fallback active.\n");
        printf("           To add a real policy, subclass FSMState and register it in FSM.\n");
    }

    void run() override
    {
        for (int i = 0; i < sc_.num_joints; i++) {
            po_.actions[i] = 0.0f;
            po_.kps[i]     = 0.0f;
            po_.kds[i]     = kd_;
        }
    }

    FSMStateName check_change() override
    {
        // Match Python LocoMode.checkChange()
        if (sc_.skill_cmd == FSMCommand::SKILL_1)   return FSMStateName::SKILL_DANCE;
        if (sc_.skill_cmd == FSMCommand::SKILL_2)   return FSMStateName::SKILL_KUNGFU;
        if (sc_.skill_cmd == FSMCommand::SKILL_3)   return FSMStateName::SKILL_KICK;
        if (sc_.skill_cmd == FSMCommand::SKILL_4)   return FSMStateName::SKILL_BEYOND_MIMIC;
        if (sc_.skill_cmd == FSMCommand::SKILL_5)   return FSMStateName::JOINT_ZERO_CHECK;
        if (sc_.skill_cmd == FSMCommand::SKILL_6)   return FSMStateName::IMU_CALIB;
        if (sc_.skill_cmd == FSMCommand::SKILL_7)   return FSMStateName::SKILL_TRACK_MIMIC;
        if (sc_.skill_cmd == FSMCommand::PASSIVE)   return FSMStateName::PASSIVE;
        return FSMStateName::LOCOMODE;
    }

private:
    float kd_;
};

// ============================================================
// Skill placeholders for parity states not fully ported yet
// ============================================================

class HoldPositionSkill : public FSMState {
public:
    HoldPositionSkill(
        FSMStateName n, const std::string& label,
        StateAndCmd& sc, PolicyOutput& po,
        float kp = 60.0f, float kd = 2.0f)
        : FSMState(n, label, sc, po), kp_(kp), kd_(kd) {}

    void enter() override
    {
        hold_q_ = sc_.q;
        printf("[%s] Entered (hold-position placeholder)\n", name_str.c_str());
    }

    void run() override
    {
        for (int i = 0; i < sc_.num_joints; i++) {
            po_.actions[i] = hold_q_[i];
            po_.kps[i]     = kp_;
            po_.kds[i]     = kd_;
        }
    }

    FSMStateName check_change() override
    {
        if (sc_.skill_cmd == FSMCommand::LOCO)      return FSMStateName::LOCOMODE;
        if (sc_.skill_cmd == FSMCommand::PASSIVE)   return FSMStateName::PASSIVE;
        if (sc_.skill_cmd == FSMCommand::POS_RESET) return FSMStateName::FIXEDPOSE;
        return name;
    }

private:
    float kp_;
    float kd_;
    std::array<float, Z1_NUM_MOTOR> hold_q_{};
};

class MimicFallbackMode : public FSMState {
public:
    MimicFallbackMode(FSMStateName n, const std::string& label,
                      StateAndCmd& sc, PolicyOutput& po,
                      float kp = 80.0f, float kd = 3.0f)
        : FSMState(n, label, sc, po), kp_(kp), kd_(kd) {}

    void enter() override
    {
        hold_q_ = sc_.q;
        printf("[%s] Entered (fallback, ONNX policy not registered)\n", name_str.c_str());
    }

    void run() override
    {
        for (int i = 0; i < sc_.num_joints; i++) {
            po_.actions[i] = hold_q_[i];
            po_.kps[i]     = kp_;
            po_.kds[i]     = kd_;
        }
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
    float kp_;
    float kd_;
    std::array<float, Z1_NUM_MOTOR> hold_q_{};
};

class SkillCooldownPlaceholder : public FSMState {
public:
    SkillCooldownPlaceholder(StateAndCmd& sc, PolicyOutput& po, float control_dt)
        : FSMState(FSMStateName::SKILL_COOLDOWN, "SkillCooldown", sc, po),
          control_dt_(std::max(1e-6f, control_dt))
    {
        num_step_ = std::max(1, (int)std::round(total_time_ / control_dt_));
    }

    void enter() override
    {
        cur_step_ = 0;
        alpha_ = 0.0f;
        for (size_t i = 0; i < SKILL_CD_UPPER_IDX.size(); i++) {
            const int motor_idx = SKILL_CD_UPPER_IDX[i];
            upper_init_[i] = sc_.q[motor_idx];
        }
        printf("[SkillCooldown] Entered\n");
    }

    void run() override
    {
        // Lower body: hold current position (C++ placeholder without cooldown NN).
        for (int i = 0; i < (int)SKILL_CD_LOWER_IDX.size(); i++) {
            const int m = SKILL_CD_LOWER_IDX[i];
            po_.actions[m] = sc_.q[m];
        }

        // Apply configured gains to all joints.
        for (int i = 0; i < sc_.num_joints; i++) {
            po_.kps[i] = SKILL_CD_KP[i];
            po_.kds[i] = SKILL_CD_KD[i];
        }

        // Upper body: interpolate to default over total_time.
        cur_step_++;
        alpha_ = std::min(1.0f, (float)cur_step_ / (float)num_step_);
        for (size_t i = 0; i < SKILL_CD_UPPER_IDX.size(); i++) {
            const int m = SKILL_CD_UPPER_IDX[i];
            po_.actions[m] = upper_init_[i] * (1.0f - alpha_) + SKILL_CD_DEFAULT[m] * alpha_;
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

private:
    float control_dt_ { DEFAULT_CTRL_DT };
    float total_time_ { 1.0f };
    int   num_step_   { 1 };
    int   cur_step_   { 0 };
    float alpha_      { 0.0f };
    std::array<float, Z1_NUM_MOTOR> upper_init_{};
};

class JointZeroCheckMode : public FSMState {
public:
    JointZeroCheckMode(StateAndCmd& sc, PolicyOutput& po, float control_dt)
        : FSMState(FSMStateName::JOINT_ZERO_CHECK, "JointZeroCheck", sc, po),
          control_dt_(std::max(1e-6f, control_dt))
    {
        hold_steps_   = std::max(1, (int)std::round(hold_time_ / control_dt_));
        settle_steps_ = std::max(0, (int)std::round(settle_time_ / control_dt_));
    }

    void enter() override
    {
        cur_joint_ = 0;
        step_in_joint_ = 0;
        printf("[JointZeroCheck] Start joint zero check.\n");
    }

    void run() override
    {
        for (int j = 0; j < sc_.num_joints; j++) {
            po_.actions[j] = JOINT_ZERO_DEFAULT[j];
            po_.kps[j]     = JOINT_ZERO_KP[j];
            po_.kds[j]     = JOINT_ZERO_KD[j];
        }

        step_in_joint_++;
        if (step_in_joint_ == settle_steps_) {
            int motor_idx = cur_joint_;
            float q = sc_.q[motor_idx];
            float target = JOINT_ZERO_DEFAULT[motor_idx];
            float offset = q - target;
            printf("[JointZeroCheck] joint_idx=%d motor_idx=%d q=%.4f target=%.4f offset=%.4f\n",
                   cur_joint_, motor_idx, q, target, offset);
        }

        if (step_in_joint_ >= hold_steps_) {
            step_in_joint_ = 0;
            cur_joint_ = (cur_joint_ + 1) % sc_.num_joints;
        }
    }

    void exit() override { printf("[JointZeroCheck] Exit joint zero check.\n"); }

    FSMStateName check_change() override
    {
        if (sc_.skill_cmd == FSMCommand::LOCO) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::LOCOMODE;
        }
        if (sc_.skill_cmd == FSMCommand::PASSIVE) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::PASSIVE;
        }
        sc_.skill_cmd = FSMCommand::INVALID;
        return FSMStateName::JOINT_ZERO_CHECK;
    }

private:
    float control_dt_  { DEFAULT_CTRL_DT };
    float hold_time_   { 1.0f };
    float settle_time_ { 0.3f };
    int   hold_steps_  { 1 };
    int   settle_steps_{ 0 };
    int   cur_joint_   { 0 };
    int   step_in_joint_{ 0 };
};

class ImuCalibMode : public FSMState {
public:
    using LocoProvider = std::function<FSMState*()>;

    ImuCalibMode(StateAndCmd& sc, PolicyOutput& po, float control_dt)
        : FSMState(FSMStateName::IMU_CALIB, "ImuCalib", sc, po),
          control_dt_(std::max(1e-6f, control_dt))
    {
        settle_steps_ = std::max(1, (int)std::round(settle_time_ / control_dt_));
        sample_steps_ = std::max(1, (int)std::round(sample_time_ / control_dt_));
    }

    void set_loco_provider(LocoProvider provider)
    {
        loco_provider_ = std::move(provider);
    }

    void enter() override
    {
        cur_step_ = 0;
        sample_count_ = 0;
        done_ = false;
        sum_q_.fill(0.0f);
        sum_g_ = {0.0f, 0.0f, 0.0f};
        sum_w_ = {0.0f, 0.0f, 0.0f};
        sc_.skill_cmd = FSMCommand::INVALID;
        printf("[ImuCalib] Start: keep standing and collect samples.\n");
    }

    void run() override
    {
        bool used_loco_pose = false;
        if (loco_provider_) {
            FSMState* loco = loco_provider_();
            if (loco) {
                // Python ImuCalib reuses LocoMode outputs for standing stabilization.
                loco->run();
                used_loco_pose = true;
            }
        }
        if (!used_loco_pose) {
            if (!warned_no_loco_) {
                printf("[ImuCalib][WARN] Loco provider missing, fallback to fixed stand pose.\n");
                warned_no_loco_ = true;
            }
            for (int i = 0; i < sc_.num_joints; i++) {
                po_.actions[i] = JOINT_ZERO_DEFAULT[i];
                po_.kps[i]     = JOINT_ZERO_KP[i];
                po_.kds[i]     = JOINT_ZERO_KD[i];
            }
        }

        cur_step_++;
        if (cur_step_ <= settle_steps_) return;

        for (int i = 0; i < sc_.num_joints; i++) sum_q_[i] += sc_.q[i];
        for (int i = 0; i < 3; i++) {
            sum_g_[i] += sc_.gravity_ori[i];
            sum_w_[i] += sc_.ang_vel[i];
        }
        sample_count_++;

        if (sample_count_ >= sample_steps_) {
            report();
            done_ = true;
        }
    }

    void exit() override { printf("[ImuCalib] Exit.\n"); }

    FSMStateName check_change() override
    {
        if (done_) {
            return FSMStateName::LOCOMODE;
        }
        if (sc_.skill_cmd == FSMCommand::PASSIVE) {
            sc_.skill_cmd = FSMCommand::INVALID;
            return FSMStateName::PASSIVE;
        }
        return FSMStateName::IMU_CALIB;
    }

private:
    void report()
    {
        const float inv_n = 1.0f / (float)std::max(1, sample_count_);
        std::array<float, Z1_NUM_MOTOR> mean_q{};
        std::array<float, 3> mean_g{}, mean_w{};
        for (int i = 0; i < sc_.num_joints; i++) mean_q[i] = sum_q_[i] * inv_n;
        for (int i = 0; i < 3; i++) {
            mean_g[i] = sum_g_[i] * inv_n;
            mean_w[i] = sum_w_[i] * inv_n;
        }

        float gx = mean_g[0], gy = mean_g[1], gz = mean_g[2];
        float g_norm = std::sqrt(gx * gx + gy * gy + gz * gz) + 1.0e-8f;
        gx /= g_norm; gy /= g_norm; gz /= g_norm;
        if (gz > 0.0f) { gx = -gx; gy = -gy; gz = -gz; }
        float roll  = std::atan2(gy, gz);
        float pitch = std::atan2(-gx, std::sqrt(gy * gy + gz * gz));

        float cr = std::cos(roll * 0.5f),  sr = std::sin(roll * 0.5f);
        float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
        std::array<float, 4> q_corr = { cp * cr, cp * sr, sp * cr, -sp * sr };

        printf("[ImuCalib] ===== Report =====\n");
        printf("[ImuCalib] mean_gravity = [%.4f, %.4f, %.4f]\n", gx, gy, gz);
        printf("[ImuCalib] mean_ang_vel = [%.4f, %.4f, %.4f]\n",
               mean_w[0], mean_w[1], mean_w[2]);
        printf("[ImuCalib] roll_offset(rad)=%.5f, pitch_offset(rad)=%.5f\n", roll, pitch);
        printf("[ImuCalib] q_correction(wxyz) = [%.6f, %.6f, %.6f, %.6f]\n",
               q_corr[0], q_corr[1], q_corr[2], q_corr[3]);
        printf("[ImuCalib] joint_offsets (|offset| > 0.1):\n");
        for (int i = 0; i < sc_.num_joints; i++) {
            float off = mean_q[i] - JOINT_ZERO_DEFAULT[i];
            if (std::fabs(off) <= 0.1f) continue;
            printf("  motor_idx=%02d offset=%.5f\n", i, off);
        }
        printf("[ImuCalib] ===== End =====\n");
    }

    float control_dt_  { DEFAULT_CTRL_DT };
    float settle_time_ { 2.0f };
    float sample_time_ { 3.0f };
    int   settle_steps_{ 1 };
    int   sample_steps_{ 1 };
    int   cur_step_    { 0 };
    int   sample_count_{ 0 };
    bool  done_        { false };
    bool  warned_no_loco_{ false };
    LocoProvider loco_provider_{};
    std::array<float, Z1_NUM_MOTOR> sum_q_{};
    std::array<float, 3> sum_g_{};
    std::array<float, 3> sum_w_{};
};

// ============================================================
// FSM — registry-based transition shell
// ============================================================

enum class FSMMode { NORMAL, CHANGE };

class FSM {
public:
    FSM(StateAndCmd& sc, PolicyOutput& po, float ctrl_dt = DEFAULT_CTRL_DT)
        : sc_(sc), po_(po),
          passive_(sc, po),
          fixed_(sc, po, ctrl_dt),
          loco_(sc, po),
          cooldown_(sc, po, ctrl_dt),
          joint_zero_(sc, po, ctrl_dt),
          imu_calib_(sc, po, ctrl_dt),
          skill_cast_(FSMStateName::SKILL_CAST, "SkillCast(stub)", sc, po),
          kungfu_(FSMStateName::SKILL_KUNGFU, "KungFu(stub)", sc, po),
          dance_(FSMStateName::SKILL_DANCE, "Dance(stub)", sc, po),
          kick_(FSMStateName::SKILL_KICK, "Kick(stub)", sc, po),
          kungfu2_(FSMStateName::SKILL_KUNGFU2, "KungFu2(stub)", sc, po),
          beyond_stub_(FSMStateName::SKILL_BEYOND_MIMIC, "BeyondMimic(stub)", sc, po),
          track_stub_(FSMStateName::SKILL_TRACK_MIMIC, "TrackMimic(stub)", sc, po)
    {
        register_builtin_policies();
        imu_calib_.set_loco_provider([this]() -> FSMState* {
            return get_registered_policy(FSMStateName::LOCOMODE);
        });
        select_state(FSMStateName::PASSIVE);
        printf("[FSM] Initialized. Policy: %s\n", cur_->name_str.c_str());
    }

    /** Register an external policy (e.g. ONNX-based). Takes ownership. */
    void register_policy(FSMStateName name, std::unique_ptr<FSMState> policy)
    {
        if (!policy) {
            printf("[FSM][WARN] Ignoring null policy for state %d\n", (int)name);
            return;
        }
        extra_[name] = std::move(policy);

        PolicyRecord rec;
        auto old = records_.find(name);
        if (old != records_.end()) rec = old->second;
        rec.policy = extra_[name].get();
        rec.mimic = rec.mimic || is_mimic_state_name(name);
        records_[name] = rec;
        aliases_[name] = name;

        printf("[FSM] registered policy %s for state %d\n",
               records_[name].policy->name_str.c_str(), (int)name);
    }

    /**
     * Directly jump to a state for shadow/test use.
     * exit() runs now; enter() is scheduled for the next run() call.
     */
    void force_state(FSMStateName name)
    {
        set_pause(false);
        if (cur_) cur_->exit();
        select_state(name);
        mode_ = FSMMode::CHANGE;
        sc_.skill_cmd = FSMCommand::INVALID;
        printf("[FSM] force_state -> %s\n", cur_->name_str.c_str());
    }

    void run()
    {
        handle_pause_command();

        if (paused_ && !is_mimic_policy()) {
            set_pause(false);
        }
        if (paused_) {
            if (sc_.skill_cmd != FSMCommand::INVALID) {
                set_pause(false);
            } else {
                sc_.skill_cmd = FSMCommand::INVALID;
            }
        }

        if (mode_ == FSMMode::CHANGE) {
            cur_->enter();
            mode_ = FSMMode::NORMAL;
            cur_->run();
            return;
        }

        cur_->run();
        if (paused_) return;

        FSMStateName requested = cur_->check_change();
        transition_if_needed(requested);
    }

    FSMState* current_policy() { return cur_; }

    FSMState* get_registered_policy(FSMStateName name)
    {
        auto state = normalize_state(name);
        auto it = records_.find(state);
        if (it == records_.end()) return nullptr;
        return it->second.policy;
    }

private:
    struct PolicyRecord {
        FSMState* policy { nullptr };
        bool mimic { false };
        std::string hint;
    };

    StateAndCmd&  sc_;
    PolicyOutput& po_;
    FSMMode mode_   { FSMMode::NORMAL };
    bool    paused_ { false };
    FSMStateName current_state_ { FSMStateName::PASSIVE };

    PassiveMode         passive_;
    FixedPose           fixed_;
    LocoModePlaceholder loco_;
    SkillCooldownPlaceholder cooldown_;
    JointZeroCheckMode      joint_zero_;
    ImuCalibMode            imu_calib_;
    HoldPositionSkill       skill_cast_;
    HoldPositionSkill       kungfu_;
    HoldPositionSkill       dance_;
    HoldPositionSkill       kick_;
    HoldPositionSkill       kungfu2_;
    MimicFallbackMode       beyond_stub_;
    MimicFallbackMode       track_stub_;

    std::map<FSMStateName, std::unique_ptr<FSMState>> extra_;
    std::map<FSMStateName, PolicyRecord> records_;
    std::map<FSMStateName, FSMStateName> aliases_;
    std::map<FSMStateName, bool> warned_missing_;

    FSMState* cur_ { nullptr };

    static bool is_mimic_state_name(FSMStateName name)
    {
        return name == FSMStateName::SKILL_BEYOND_MIMIC
            || name == FSMStateName::SKILL_TRACK_MIMIC;
    }

    void register_builtin(
        FSMStateName name,
        FSMState* policy,
        bool mimic = false,
        const std::string& hint = "")
    {
        records_[name] = PolicyRecord{policy, mimic, hint};
        aliases_[name] = name;
    }

    void register_builtin_policies()
    {
        register_builtin(
            FSMStateName::PASSIVE,
            &passive_,
            false,
            "[Hints] PASSIVE/DAMPING, START=POS_RESET, R1+A=LOCO");
        register_builtin(
            FSMStateName::FIXEDPOSE,
            &fixed_,
            false,
            "[Hints] R1+A=LOCO, L3=PASSIVE");
        register_builtin(
            FSMStateName::LOCOMODE,
            &loco_,
            false,
            "[Hints] R1+X/R1+Y/L1+Y=skill, L3=PASSIVE");
        register_builtin(
            FSMStateName::SKILL_COOLDOWN,
            &cooldown_,
            false,
            "[Hints] auto return to LOCO or L3=PASSIVE");
        register_builtin(FSMStateName::SKILL_CAST, &skill_cast_);
        register_builtin(FSMStateName::SKILL_KUNGFU, &kungfu_);
        register_builtin(FSMStateName::SKILL_DANCE, &dance_);
        register_builtin(FSMStateName::SKILL_KICK, &kick_);
        register_builtin(FSMStateName::SKILL_KUNGFU2, &kungfu2_);
        register_builtin(
            FSMStateName::SKILL_BEYOND_MIMIC,
            &beyond_stub_,
            true,
            "[Hints] R1+A=LOCO, L3=PASSIVE, UP=PAUSE");
        register_builtin(
            FSMStateName::SKILL_TRACK_MIMIC,
            &track_stub_,
            true,
            "[Hints] R1+A=LOCO, L3=PASSIVE, UP=PAUSE");
        register_builtin(
            FSMStateName::JOINT_ZERO_CHECK,
            &joint_zero_,
            false,
            "[Hints] joint-zero check, R1+A=LOCO, L3=PASSIVE");
        register_builtin(
            FSMStateName::IMU_CALIB,
            &imu_calib_,
            false,
            "[Hints] auto return to LOCO or L3=PASSIVE");
    }

    void handle_pause_command()
    {
        if (sc_.skill_cmd != FSMCommand::PAUSE) return;
        sc_.skill_cmd = FSMCommand::INVALID;
        if (is_mimic_policy()) set_pause(!paused_);
        else                   set_pause(false);
    }

    void transition_if_needed(FSMStateName requested)
    {
        if (requested == FSMStateName::INVALID) requested = current_state_;

        FSMStateName target_state = normalize_state(requested);
        FSMState* target = get_registered_policy(target_state);
        if (!target) return;
        if (target == cur_ && target_state == current_state_) return;

        set_pause(false);
        mode_ = FSMMode::CHANGE;
        cur_->exit();
        current_state_ = target_state;
        cur_ = target;
        printf("[FSM] -> %s\n", cur_->name_str.c_str());
        print_mode_hints(target_state);
    }

    FSMStateName normalize_state(FSMStateName requested)
    {
        auto alias = aliases_.find(requested);
        if (alias != aliases_.end()) return alias->second;

        if (!warned_missing_[requested]) {
            warned_missing_[requested] = true;
            printf("[FSM][WARN] Unknown/unregistered state %d, keeping %s\n",
                   (int)requested, cur_ ? cur_->name_str.c_str() : "null");
        }
        return current_state_;
    }

    bool select_state(FSMStateName requested)
    {
        FSMStateName target_state = normalize_state(requested);
        FSMState* target = get_registered_policy(target_state);
        if (!target) {
            if (!cur_) {
                current_state_ = FSMStateName::PASSIVE;
                cur_ = &passive_;
            }
            return false;
        }
        current_state_ = target_state;
        cur_ = target;
        return true;
    }

    bool is_mimic_policy() const
    {
        auto it = records_.find(current_state_);
        return it != records_.end() && it->second.mimic;
    }

    void set_pause(bool enable)
    {
        if (paused_ == enable && sc_.pause == enable) return;
        paused_ = enable;
        sc_.pause = enable;
        if (enable) printf("[FSM] Pause ON\n");
        else        printf("[FSM] Pause OFF\n");
    }

    void print_mode_hints(FSMStateName state)
    {
        auto it = records_.find(state);
        if (it != records_.end() && !it->second.hint.empty()) {
            printf("%s\n", it->second.hint.c_str());
        }
    }
};

// ============================================================
// Controller — mirrors Python Controller class
// ============================================================

class Controller {
public:
    explicit Controller(const Config& cfg)
        : config_(cfg),
          state_cmd_(cfg.num_joints),
          policy_output_(cfg.num_joints),
          fsm_(state_cmd_, policy_output_, cfg.control_dt),
          safety_(cfg.num_joints, cfg),
          cmd_gate_(cfg.command_hold_frames),
          cmd_pub_(cfg.lowcmd_topic),
          state_sub_(cfg.lowstate_topic)
    {
        cmd_pub_.InitChannel();

        state_sub_.InitChannel(
            [this](const void* msg) { this->low_state_handler(msg); }, 10);

        if (config_.wait_for_lowstate_on_init) {
            wait_for_low_state();
        } else {
            printf("[Controller] Init without blocking for LowState (shadow prewarm).\n");
        }
        init_cmd();

        printf("[Controller] Ready — %d joints @ %.0f Hz, safety=%s\n",
               cfg.num_joints, 1.0f / cfg.control_dt,
               cfg.safety_enable ? "ON" : "OFF");
    }

    /** Send zero-torque commands until START is pressed. */
    void zero_torque_state()
    {
        printf("[Controller] Zero-torque mode. Press START to continue...\n");
        while (true) {
            if (remote_.is_pressed(KEY_START)) break;
            create_zero_cmd();
            send_cmd();
            sleep_sec(config_.control_dt);
        }
    }

    /** One control step — call this in the main loop. */
    void run()
    {
        double t0 = now_sec();

        if (config_.sync_on_lowstate) {
            uint32_t tick_now = 0;
            {
                std::shared_lock<std::shared_mutex> lk(state_mtx_);
                tick_now = low_state_.tick();
            }
            if (tick_now == last_lowstate_tick_) {
                sleep_sec(std::max(0.001, (double)config_.control_dt * 0.25));
                return;
            }
            last_lowstate_tick_ = tick_now;
        }

        // ── button → FSM command mapping ─────────────────────────────
        if (remote_.is_pressed(KEY_F1))
            state_cmd_.skill_cmd = FSMCommand::PASSIVE;
        if (remote_.is_released(KEY_UP))
            state_cmd_.skill_cmd = FSMCommand::PAUSE;
        if (cmd_gate_.trigger("POS_RESET", remote_.is_pressed(KEY_START)))
            state_cmd_.skill_cmd = FSMCommand::POS_RESET;
        if (cmd_gate_.trigger("LOCO",
                remote_.is_pressed(KEY_A) && remote_.is_pressed(KEY_R1)))
            state_cmd_.skill_cmd = FSMCommand::LOCO;
        if (cmd_gate_.trigger("SKILL_1",
                remote_.is_pressed(KEY_X) && remote_.is_pressed(KEY_R1)))
            state_cmd_.skill_cmd = FSMCommand::SKILL_1;
        if (cmd_gate_.trigger("SKILL_2",
                remote_.is_pressed(KEY_Y) && remote_.is_pressed(KEY_R1)))
            state_cmd_.skill_cmd = FSMCommand::SKILL_2;
        if (cmd_gate_.trigger("SKILL_4",
                remote_.is_pressed(KEY_Y) && remote_.is_pressed(KEY_L1)))
            state_cmd_.skill_cmd = FSMCommand::SKILL_4;
        if (cmd_gate_.trigger("SKILL_6",
                remote_.is_pressed(KEY_X) && remote_.is_pressed(KEY_L1)))
            state_cmd_.skill_cmd = FSMCommand::SKILL_6;
        if (cmd_gate_.trigger("SKILL_7",
                remote_.is_pressed(KEY_A) && remote_.is_pressed(KEY_L1)))
            state_cmd_.skill_cmd = FSMCommand::SKILL_7;

        state_cmd_.vel_cmd[0] =  remote_.ly;
        state_cmd_.vel_cmd[1] = -remote_.lx;
        state_cmd_.vel_cmd[2] = -remote_.rx;

        // ── read robot state ─────────────────────────────────────────
        uint32_t state_tick = 0;
        {
            std::shared_lock<std::shared_mutex> lk(state_mtx_);
            for (int i = 0; i < config_.num_joints; i++) {
                state_cmd_.q[i]  = low_state_.motor_state()[i].q();
                state_cmd_.dq[i] = low_state_.motor_state()[i].dq();
            }
            const auto& quat = low_state_.imu_state().quaternion();
            state_cmd_.base_quat = { quat[0], quat[1], quat[2], quat[3] };
            const auto& gyro = low_state_.imu_state().gyroscope();
            state_cmd_.ang_vel = { gyro[0], gyro[1], gyro[2] };
            state_tick = low_state_.tick();
        }
        state_cmd_.policy_step_override = config_.sync_on_lowstate
            ? std::max(0, (int)state_tick - 1)
            : -1;
        state_cmd_.gravity_ori = get_gravity_orientation(state_cmd_.base_quat);

        // ── policy ───────────────────────────────────────────────────
        fsm_.run();

        // ── safety filter ────────────────────────────────────────────
        auto actions = policy_output_.actions;
        auto kps     = policy_output_.kps;
        auto kds     = policy_output_.kds;
        bool force_damping = safety_.filter_actions(actions, kps, kds);

        // ── build low-level command ──────────────────────────────────
        if (force_damping) {
            create_damping_cmd();
        } else {
            low_cmd_.mode_pr()      = 0;   // PR mode for Z1
            low_cmd_.mode_machine() = mode_machine_;
            for (int i = 0; i < config_.num_joints; i++) {
                low_cmd_.motor_cmd()[i].mode() = 1;
                low_cmd_.motor_cmd()[i].q()    = actions[i];
                low_cmd_.motor_cmd()[i].dq()   = 0.0f;
                low_cmd_.motor_cmd()[i].kp()   = kps[i];
                low_cmd_.motor_cmd()[i].kd()   = kds[i];
                low_cmd_.motor_cmd()[i].tau()  = 0.0f;
            }
        }
        send_cmd();

        // ── timing ───────────────────────────────────────────────────
        double elapsed = now_sec() - t0;
        double remain  = config_.control_dt - elapsed;
        if (remain > 0) {
            sleep_sec(remain);
            counter_over_time_ = 0;
        } else {
            printf("[Controller] Loop overtime (%.2f ms over)\n", -remain * 1e3);
            if (++counter_over_time_ >= config_.error_over_time)
                safety_.report_fault("control loop overtime");
        }
    }

    bool is_exit_requested() const { return remote_.is_pressed(KEY_SELECT); }

    void send_damping_and_exit()
    {
        create_damping_cmd();
        send_cmd();
        printf("[Controller] Exit — damping applied to all motors.\n");
    }

    /** Expose FSM so caller can register external policies. */
    FSM& get_fsm() { return fsm_; }

    /** Expose state/output for external policy construction. */
    StateAndCmd&  get_state_cmd()     { return state_cmd_; }
    PolicyOutput& get_policy_output() { return policy_output_; }

private:
    // ── DDS callback (runs in subscriber thread) ──────────────────
    void low_state_handler(const void* msg)
    {
        const auto* state = static_cast<const LowState_*>(msg);
        std::unique_lock<std::shared_mutex> lk(state_mtx_);
        low_state_    = *state;
        mode_machine_ = state->mode_machine();
        remote_.set(state->wireless_remote());
    }

    void wait_for_low_state()
    {
        printf("[Controller] Waiting for robot connection...\n");
        while (true) {
            {
                std::shared_lock<std::shared_mutex> lk(state_mtx_);
                if (low_state_.tick() != 0) break;
            }
            sleep_sec(config_.control_dt);
        }
        printf("[Controller] Connected (tick=%u)\n", low_state_.tick());
    }

    void init_cmd()
    {
        low_cmd_.mode_pr()      = 0;
        low_cmd_.mode_machine() = mode_machine_;
        for (int i = 0; i < 35; i++) {
            low_cmd_.motor_cmd()[i].mode() = 1;
            low_cmd_.motor_cmd()[i].q()    = 0.f;
            low_cmd_.motor_cmd()[i].dq()   = 0.f;
            low_cmd_.motor_cmd()[i].kp()   = 0.f;
            low_cmd_.motor_cmd()[i].kd()   = 0.f;
            low_cmd_.motor_cmd()[i].tau()  = 0.f;
        }
    }

    void create_damping_cmd()
    {
        low_cmd_.mode_pr()      = 0;
        low_cmd_.mode_machine() = mode_machine_;
        for (int i = 0; i < 35; i++) {
            low_cmd_.motor_cmd()[i].mode() = 1;
            low_cmd_.motor_cmd()[i].q()    = 0.f;
            low_cmd_.motor_cmd()[i].dq()   = 0.f;
            low_cmd_.motor_cmd()[i].kp()   = 0.f;
            low_cmd_.motor_cmd()[i].kd()   = config_.damping_kd;
            low_cmd_.motor_cmd()[i].tau()  = 0.f;
        }
    }

    void create_zero_cmd()
    {
        for (int i = 0; i < 35; i++) {
            low_cmd_.motor_cmd()[i].q()   = 0.f;
            low_cmd_.motor_cmd()[i].dq()  = 0.f;
            low_cmd_.motor_cmd()[i].kp()  = 0.f;
            low_cmd_.motor_cmd()[i].kd()  = 0.f;
            low_cmd_.motor_cmd()[i].tau() = 0.f;
        }
    }

    void send_cmd()
    {
        if (config_.safety_dry_run) return;
        low_cmd_.crc() = crc32_core(
            reinterpret_cast<const uint32_t*>(&low_cmd_),
            (sizeof(low_cmd_) >> 2) - 1);
        cmd_pub_.Write(low_cmd_);
    }

    // ── members ──────────────────────────────────────────────────
    Config         config_;
    RemoteController remote_;
    StateAndCmd    state_cmd_;
    PolicyOutput   policy_output_;
    FSM            fsm_;
    SafetyFilter   safety_;
    HoldToConfirm  cmd_gate_;

    LowState_ low_state_;
    LowCmd_   low_cmd_;
    uint8_t   mode_machine_      { 0 };
    uint32_t  last_lowstate_tick_{ 0 };
    int       counter_over_time_ { 0 };

    ChannelPublisher <LowCmd_>   cmd_pub_;
    ChannelSubscriber<LowState_> state_sub_;
    std::shared_mutex            state_mtx_;
};

// ============================================================
// Optional: BeyondMimic ONNX policy
// Compiled only when cmake target robot_controller_onnx is built
// (i.e. -DENABLE_BEYOND_MIMIC=1 is defined).
// ============================================================

#ifdef ENABLE_BEYOND_MIMIC
#  include "beyond_mimic_policy.h"
#  include "onnx_skill_policies.h"
#endif

// ============================================================
// Signal handler
// ============================================================

static std::atomic<bool> g_exit { false };

static void sig_handler(int) { g_exit = true; }

// ============================================================
// main
// ============================================================

int main(int argc, char* argv[])
{
    // ── argument parsing ─────────────────────────────────────────
    // Usage:
    //   robot_controller_onnx [net_iface] [num_joints]
    //               [--yaml /path/to/BeyondMimic.yaml]
    //               [--track-yaml /path/to/TrackMimic.yaml]
    Config config;
    std::string yaml_path;
    std::string track_yaml_path;
    std::string shadow_state = "beyond";
    bool shadow = false;   // --shadow: skip zero_torque_state() wait (for MuJoCo comparison)

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--yaml" && i+1 < argc) {
            yaml_path = argv[++i];
        } else if (arg == "--track-yaml" && i+1 < argc) {
            track_yaml_path = argv[++i];
        } else if (arg == "--shadow-state" && i+1 < argc) {
            shadow_state = argv[++i];
            std::transform(shadow_state.begin(), shadow_state.end(), shadow_state.begin(),
                           [](unsigned char ch) { return (char)std::tolower(ch); });
        } else if (arg == "--net" && i+1 < argc) {
            config.net = argv[++i];
        } else if (arg == "--joints" && i+1 < argc) {
            config.num_joints = std::atoi(argv[++i]);
        } else if (arg == "--safety") {
            config.safety_enable = true;
        } else if (arg == "--dry-run") {
            config.safety_dry_run = true;
        } else if (arg == "--sync-lowstate") {
            config.sync_on_lowstate = true;
        } else if (arg == "--shadow") {
            shadow = true;              // skip physical start-up sequence
        } else if (i == 1 && arg[0] != '-') {
            config.net = arg;           // positional arg 1: interface
        } else if (i == 2 && arg[0] != '-') {
            config.num_joints = std::atoi(arg.c_str());  // positional arg 2: joints
        }
    }
    if (shadow) {
        // In shadow-compare, avoid blocking constructor on first LowState.
        // This lets policies/model load run in parallel with MuJoCo startup.
        config.wait_for_lowstate_on_init = false;
    }

    printf("=== RoboMimic Robot Controller (C++ ONNX/DDS) ===\n");
    printf("Interface : %s\n", config.net.c_str());
    printf("Joints    : %d\n", config.num_joints);
    printf("Control dt: %.0f ms  (%.0f Hz)\n",
           config.control_dt * 1e3f, 1.0f / config.control_dt);
    printf("Safety    : %s\n", config.safety_enable ? "enabled" : "disabled");
    printf("Shadow    : %s\n", shadow ? "yes (no zero_torque wait)" : "no");
    printf("SyncTick  : %s\n", config.sync_on_lowstate ? "yes" : "no");
    printf("WaitState : %s\n", config.wait_for_lowstate_on_init ? "yes" : "no (shadow prewarm)");
#ifdef ENABLE_BEYOND_MIMIC
    printf("ONNX yaml : %s\n", yaml_path.empty() ? "(none)" : yaml_path.c_str());
    printf("Track yaml: %s\n", track_yaml_path.empty() ? "(none)" : track_yaml_path.c_str());
    printf("Shadow FSM: %s\n", shadow_state.c_str());
#endif
    printf("\n");

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    // ── initialise DDS ───────────────────────────────────────────
    ChannelFactory::Instance()->Init(0, config.net);

    Controller controller(config);

    // ── register ONNX policy (if compiled + yaml provided) ───────
#ifdef ENABLE_BEYOND_MIMIC
    {
        namespace fs = std::filesystem;
        fs::path policy_root;
        if (!yaml_path.empty()) {
            try {
                fs::path yp = fs::absolute(fs::path(yaml_path));
                // .../policies/beyond_mimic/config/BeyondMimic.yaml -> .../policies
                auto p0 = yp.parent_path();        // config
                auto p1 = p0.parent_path();        // beyond_mimic
                auto p2 = p1.parent_path();        // policies
                if (fs::exists(p2)) policy_root = p2;
            } catch (...) {}
        }
        if (policy_root.empty()) {
            policy_root = fs::current_path() / "policies";
        }

        auto try_register = [&](FSMStateName name, auto&& make_policy, const char* label) {
            try {
                controller.get_fsm().register_policy(name, make_policy());
                printf("[Main] %s registered\n", label);
            } catch (const std::exception& e) {
                printf("[Main][WARN] %s load failed: %s\n", label, e.what());
            }
        };

        const fs::path loco_yaml = policy_root / "loco_mode" / "config" / "LocoMode_lowKp.yaml";
        if (fs::exists(loco_yaml)) {
            try_register(
                FSMStateName::LOCOMODE,
                [&]() {
                    return std::make_unique<onnx_skill::LocoModePolicy>(
                        controller.get_state_cmd(), controller.get_policy_output(), loco_yaml.string());
                },
                "LocoMode");
        }

        const fs::path dance_yaml = policy_root / "dance" / "config" / "Dance.yaml";
        if (fs::exists(dance_yaml)) {
            try_register(
                FSMStateName::SKILL_DANCE,
                [&]() {
                    return std::make_unique<onnx_skill::MotionPolicy>(
                        FSMStateName::SKILL_DANCE, "Dance",
                        controller.get_state_cmd(), controller.get_policy_output(),
                        dance_yaml.string(),
                        /*update_history_before_obs=*/true,
                        /*clip_action10=*/false);
                },
                "Dance");
        }

        const fs::path kungfu_yaml = policy_root / "kungfu" / "config" / "KungFu.yaml";
        if (fs::exists(kungfu_yaml)) {
            try_register(
                FSMStateName::SKILL_KUNGFU,
                [&]() {
                    return std::make_unique<onnx_skill::MotionPolicy>(
                        FSMStateName::SKILL_KUNGFU, "KungFu",
                        controller.get_state_cmd(), controller.get_policy_output(),
                        kungfu_yaml.string(),
                        /*update_history_before_obs=*/false,
                        /*clip_action10=*/true);
                },
                "KungFu");
        }

        const fs::path kick_yaml = policy_root / "kick" / "config" / "Kick.yaml";
        if (fs::exists(kick_yaml)) {
            try_register(
                FSMStateName::SKILL_KICK,
                [&]() {
                    return std::make_unique<onnx_skill::MotionPolicy>(
                        FSMStateName::SKILL_KICK, "Kick",
                        controller.get_state_cmd(), controller.get_policy_output(),
                        kick_yaml.string(),
                        /*update_history_before_obs=*/false,
                        /*clip_action10=*/false);
                },
                "Kick");
        }

        const fs::path kungfu2_yaml = policy_root / "kungfu2" / "config" / "KungFu2.yaml";
        if (fs::exists(kungfu2_yaml)) {
            try_register(
                FSMStateName::SKILL_KUNGFU2,
                [&]() {
                    return std::make_unique<onnx_skill::MotionPolicy>(
                        FSMStateName::SKILL_KUNGFU2, "KungFu2",
                        controller.get_state_cmd(), controller.get_policy_output(),
                        kungfu2_yaml.string(),
                        /*update_history_before_obs=*/false,
                        /*clip_action10=*/true);
                },
                "KungFu2");
        }

        const fs::path cooldown_yaml = policy_root / "skill_cooldown" / "config" / "SkillCooldown.yaml";
        if (fs::exists(cooldown_yaml)) {
            try_register(
                FSMStateName::SKILL_COOLDOWN,
                [&]() {
                    return std::make_unique<onnx_skill::SkillCooldownPolicy>(
                        controller.get_state_cmd(), controller.get_policy_output(), cooldown_yaml.string());
                },
                "SkillCooldown");
        }

        const fs::path cast_yaml = policy_root / "skill_cast" / "config" / "SkillCast.yaml";
        if (fs::exists(cast_yaml)) {
            try_register(
                FSMStateName::SKILL_CAST,
                [&]() {
                    return std::make_unique<onnx_skill::SkillCastPolicy>(
                        controller.get_state_cmd(), controller.get_policy_output(), cast_yaml.string());
                },
                "SkillCast");
        }
    }

    if (!yaml_path.empty()) {
        try {
            auto policy = std::make_unique<BeyondMimicPolicy>(
                controller.get_state_cmd(),
                controller.get_policy_output(),
                yaml_path,
                config.control_dt);
            controller.get_fsm().register_policy(
                FSMStateName::SKILL_BEYOND_MIMIC, std::move(policy));
            printf("[Main] BeyondMimic policy registered "
                   "(trigger from LOCO: FSMCommand::SKILL_4 / Y+L1)\n");
        } catch (const std::exception& e) {
            printf("[Main][WARN] BeyondMimic load failed: %s\n", e.what());
            printf("             Continuing without ONNX policies.\n");
        }
    }

    if (!track_yaml_path.empty()) {
        try {
            auto track_policy = std::make_unique<BeyondMimicPolicy>(
                controller.get_state_cmd(),
                controller.get_policy_output(),
                track_yaml_path,
                config.control_dt,
                FSMStateName::SKILL_TRACK_MIMIC,
                "TrackMimic",
                /*require_motion_file=*/true);
            controller.get_fsm().register_policy(
                FSMStateName::SKILL_TRACK_MIMIC, std::move(track_policy));
            printf("[Main] TrackMimic policy registered "
                   "(trigger from LOCO: FSMCommand::SKILL_7 / A+L1)\n");
        } catch (const std::exception& e) {
            printf("[Main][WARN] TrackMimic load failed: %s\n", e.what());
            printf("             Continuing with fallback TrackMimic stub.\n");
        }
    }
#endif

    // ── zero-torque until operator presses START ─────────────────
    if (shadow) {
        printf("[Main] Shadow mode: skipping zero_torque_state().\n");
#ifdef ENABLE_BEYOND_MIMIC
        // Auto-arm: jump directly to SKILL_BEYOND_MIMIC so we start comparing
        // as soon as MuJoCo starts sending LowState.
        if (shadow_state == "track" && !track_yaml_path.empty()) {
            printf("[Main] Shadow mode: auto-arming SKILL_TRACK_MIMIC.\n");
            controller.get_fsm().force_state(FSMStateName::SKILL_TRACK_MIMIC);
        } else if (!yaml_path.empty()) {
            printf("[Main] Shadow mode: auto-arming SKILL_BEYOND_MIMIC.\n");
            controller.get_fsm().force_state(FSMStateName::SKILL_BEYOND_MIMIC);
        }
#endif
    } else {
        controller.zero_torque_state();
    }

    // ── main 50 Hz control loop ───────────────────────────────────
    printf("[Main] Control loop started. Press SELECT to exit.\n");
    while (!g_exit) {
        controller.run();
        if (controller.is_exit_requested()) break;
    }

    controller.send_damping_and_exit();
    printf("[Main] Done.\n");
    return 0;
}
