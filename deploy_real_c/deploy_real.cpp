/**
 * deploy_real.cpp
 * C++ port of deploy_real/deploy_real.py for the Unitree G1 robot.
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
 *   ./deploy_real [network_interface] [num_joints]
 *   e.g.  ./deploy_real enp4s0 29
 *
 * Build: see CMakeLists.txt
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <cfloat>
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
#include <time.h>
#include <unistd.h>
#include <signal.h>

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

static constexpr int    G1_NUM_MOTOR    = 29;
static constexpr float  DEFAULT_CTRL_DT = 0.02f;   // 50 Hz
static constexpr int    DEFAULT_ERR_OT  = 5;
static constexpr float  DAMPING_KD      = 8.0f;

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
    int         num_joints      = G1_NUM_MOTOR;
    std::string lowcmd_topic    = "rt/lowcmd";
    std::string lowstate_topic  = "rt/lowstate";
    float       control_dt      = DEFAULT_CTRL_DT;
    int         error_over_time = DEFAULT_ERR_OT;

    // safety.yaml
    bool  safety_enable           = false;
    bool  safety_dry_run          = false;
    bool  safety_log_events       = true;
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

    std::array<float, G1_NUM_MOTOR> q   {};
    std::array<float, G1_NUM_MOTOR> dq  {};
    std::array<float, G1_NUM_MOTOR> ddq {};
    std::array<float, G1_NUM_MOTOR> tau_est {};

    std::array<float, 3> gravity_ori { 0.f, 0.f, -1.f };
    std::array<float, 3> ang_vel     {};
    std::array<float, 4> base_quat   { 1.f, 0.f, 0.f, 0.f };  // w,x,y,z
    std::array<float, 3> vel_cmd     {};

    FSMCommand skill_cmd { FSMCommand::INVALID };
    bool pause { false };

    explicit StateAndCmd(int n) : num_joints(n) {}
};

// ============================================================
// PolicyOutput — mirrors Python PolicyOutput
// ============================================================

struct PolicyOutput {
    int num_joints;
    std::array<float, G1_NUM_MOTOR> actions {};
    std::array<float, G1_NUM_MOTOR> kps     {};
    std::array<float, G1_NUM_MOTOR> kds     {};

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
        std::array<float, G1_NUM_MOTOR>& actions,
        std::array<float, G1_NUM_MOTOR>& kps,
        std::array<float, G1_NUM_MOTOR>& kds)
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

    std::array<float, G1_NUM_MOTOR> prev_action_{};
    std::array<float, G1_NUM_MOTOR> prev_kp_    {};
    std::array<float, G1_NUM_MOTOR> prev_kd_    {};

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
        if (sc_.skill_cmd == FSMCommand::POS_RESET) return FSMStateName::FIXEDPOSE;
        if (sc_.skill_cmd == FSMCommand::LOCO)      return FSMStateName::LOCOMODE;
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
    // Default target angles for G1 (all zeros; adjust per your robot)
    std::array<float, G1_NUM_MOTOR> default_pos{};

    FixedPose(StateAndCmd& sc, PolicyOutput& po,
              float dt = DEFAULT_CTRL_DT, float kp = 100.f, float kd = 3.f)
        : FSMState(FSMStateName::FIXEDPOSE, "FixedPose", sc, po),
          control_dt_(dt), kp_(kp), kd_(kd) {}

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
            po_.actions[i] = init_pos_[i] * (1.0f - ratio) + default_pos[i] * ratio;
            po_.kps[i]     = kp_;
            po_.kds[i]     = kd_;
        }
    }

    FSMStateName check_change() override
    {
        if (sc_.skill_cmd == FSMCommand::PASSIVE)   return FSMStateName::PASSIVE;
        if (sc_.skill_cmd == FSMCommand::LOCO)      return FSMStateName::LOCOMODE;
        return FSMStateName::FIXEDPOSE;
    }

private:
    float control_dt_;
    float kp_, kd_;
    float duration_ { 2.0f };
    float elapsed_  { 0.0f };
    std::array<float, G1_NUM_MOTOR> init_pos_{};
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
        if (sc_.skill_cmd == FSMCommand::PASSIVE)   return FSMStateName::PASSIVE;
        if (sc_.skill_cmd == FSMCommand::POS_RESET) return FSMStateName::FIXEDPOSE;
        // SKILL_1 / SKILL_2 (Y+R1 / X+R1) → activate registered ONNX policy
        if (sc_.skill_cmd == FSMCommand::SKILL_1 ||
            sc_.skill_cmd == FSMCommand::SKILL_2)   return FSMStateName::SKILL_BEYOND_MIMIC;
        return FSMStateName::LOCOMODE;
    }

private:
    float kd_;
};

// ============================================================
// FSM — mirrors Python FSM
// ============================================================

enum class FSMMode { NORMAL, CHANGE };

class FSM {
public:
    FSM(StateAndCmd& sc, PolicyOutput& po, float ctrl_dt = DEFAULT_CTRL_DT)
        : sc_(sc), po_(po),
          passive_(sc, po),
          fixed_(sc, po, ctrl_dt),
          loco_(sc, po)
    {
        cur_ = &passive_;
        printf("[FSM] Initialized. Policy: %s\n", cur_->name_str.c_str());
    }

    /** Register an external policy (e.g. ONNX-based). Takes ownership. */
    void register_policy(FSMStateName name, std::unique_ptr<FSMState> policy)
    {
        extra_[name] = std::move(policy);
    }

    /**
     * Directly jump to a state — for shadow / test use only.
     * Calls exit() on the current state, then schedules enter() on the next
     * run() call (via CHANGE mode), exactly like a normal FSM transition.
     */
    void force_state(FSMStateName name)
    {
        if (cur_) cur_->exit();
        set_policy(name);
        mode_ = FSMMode::CHANGE;   // enter() will be called on the next run()
        printf("[FSM] force_state → %s\n", cur_->name_str.c_str());
    }

    void run()
    {
        // PAUSE command
        if (sc_.skill_cmd == FSMCommand::PAUSE) {
            sc_.skill_cmd = FSMCommand::INVALID;
        }

        if (mode_ == FSMMode::NORMAL) {
            cur_->run();
            if (!paused_) {
                FSMStateName next = cur_->check_change();
                if (next != cur_->name) {
                    mode_ = FSMMode::CHANGE;
                    cur_->exit();
                    set_policy(next);
                    printf("[FSM] → %s\n", cur_->name_str.c_str());
                }
            }
        } else {  // CHANGE
            cur_->enter();
            mode_ = FSMMode::NORMAL;
            cur_->run();
        }

        sc_.skill_cmd = FSMCommand::INVALID;
    }

private:
    StateAndCmd&  sc_;
    PolicyOutput& po_;
    FSMMode mode_   { FSMMode::NORMAL };
    bool    paused_ { false };

    PassiveMode         passive_;
    FixedPose           fixed_;
    LocoModePlaceholder loco_;

    std::map<FSMStateName, std::unique_ptr<FSMState>> extra_;

    FSMState* cur_ { nullptr };

    void set_policy(FSMStateName name)
    {
        if (extra_.count(name)) { cur_ = extra_[name].get(); return; }
        switch (name) {
            case FSMStateName::PASSIVE:   cur_ = &passive_; break;
            case FSMStateName::FIXEDPOSE: cur_ = &fixed_;   break;
            case FSMStateName::LOCOMODE:  cur_ = &loco_;    break;
            default:
                printf("[FSM] Unknown state %d, falling back to PASSIVE\n", (int)name);
                cur_ = &passive_;
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
            [this](const void* msg) { this->low_state_handler(msg); }, 1);

        wait_for_low_state();
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
        }
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
            low_cmd_.mode_pr()      = 0;   // PR mode for G1
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
    int       counter_over_time_ { 0 };

    ChannelPublisher <LowCmd_>   cmd_pub_;
    ChannelSubscriber<LowState_> state_sub_;
    std::shared_mutex            state_mtx_;
};

// ============================================================
// Optional: BeyondMimic ONNX policy
// Compiled only when cmake target deploy_real_onnx is built
// (i.e. -DENABLE_BEYOND_MIMIC=1 is defined).
// ============================================================

#ifdef ENABLE_BEYOND_MIMIC
#  include "beyond_mimic_policy.h"
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
    // Usage: deploy_real [net_iface] [num_joints] [--yaml /path/to/Policy.yaml]
    Config config;
    std::string yaml_path;
    bool shadow = false;   // --shadow: skip zero_torque_state() wait (for MuJoCo comparison)

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--yaml" && i+1 < argc) {
            yaml_path = argv[++i];
        } else if (arg == "--net" && i+1 < argc) {
            config.net = argv[++i];
        } else if (arg == "--joints" && i+1 < argc) {
            config.num_joints = std::atoi(argv[++i]);
        } else if (arg == "--safety") {
            config.safety_enable = true;
        } else if (arg == "--dry-run") {
            config.safety_dry_run = true;
        } else if (arg == "--shadow") {
            shadow = true;              // skip physical start-up sequence
        } else if (i == 1 && arg[0] != '-') {
            config.net = arg;           // positional arg 1: interface
        } else if (i == 2 && arg[0] != '-') {
            config.num_joints = std::atoi(arg.c_str());  // positional arg 2: joints
        }
    }

    printf("=== RoboMimic Deploy Real (C++ version) ===\n");
    printf("Interface : %s\n", config.net.c_str());
    printf("Joints    : %d\n", config.num_joints);
    printf("Control dt: %.0f ms  (%.0f Hz)\n",
           config.control_dt * 1e3f, 1.0f / config.control_dt);
    printf("Safety    : %s\n", config.safety_enable ? "enabled" : "disabled");
    printf("Shadow    : %s\n", shadow ? "yes (no zero_torque wait)" : "no");
#ifdef ENABLE_BEYOND_MIMIC
    printf("ONNX yaml : %s\n", yaml_path.empty() ? "(none)" : yaml_path.c_str());
#endif
    printf("\n");

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    // ── initialise DDS ───────────────────────────────────────────
    ChannelFactory::Instance()->Init(0, config.net);

    Controller controller(config);

    // ── register ONNX policy (if compiled + yaml provided) ───────
#ifdef ENABLE_BEYOND_MIMIC
    if (!yaml_path.empty()) {
        try {
            auto policy = std::make_unique<BeyondMimicPolicy>(
                controller.get_state_cmd(),
                controller.get_policy_output(),
                yaml_path);
            controller.get_fsm().register_policy(
                FSMStateName::SKILL_BEYOND_MIMIC, std::move(policy));
            printf("[Main] BeyondMimic policy registered "
                   "(trigger: R1+B not bound; use FSMCommand::SKILL_2 / Y+R1)\n");
        } catch (const std::exception& e) {
            printf("[Main][WARN] BeyondMimic load failed: %s\n", e.what());
            printf("             Continuing without ONNX policy.\n");
        }
    }
#endif

    // ── zero-torque until operator presses START ─────────────────
    if (shadow) {
        printf("[Main] Shadow mode: skipping zero_torque_state().\n");
#ifdef ENABLE_BEYOND_MIMIC
        // Auto-arm: jump directly to SKILL_BEYOND_MIMIC so we start comparing
        // as soon as MuJoCo starts sending LowState.
        if (!yaml_path.empty()) {
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
