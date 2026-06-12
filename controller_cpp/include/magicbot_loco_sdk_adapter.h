#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include "magic_robot.h"
#include "magicbot_loco_core.h"

namespace magicbot_loco {

class SdkRobotState {
public:
    void update_leg(const std::shared_ptr<magic::z1::JointState> msg);
    void update_arm(const std::shared_ptr<magic::z1::JointState> msg);
    void update_waist(const std::shared_ptr<magic::z1::JointState> msg);
    void update_head(const std::shared_ptr<magic::z1::JointState> msg);
    void update_imu(const std::shared_ptr<magic::z1::Imu> msg);

    RobotSnapshot snapshot() const;
    double state_age_ms() const;
    std::pair<bool, std::string> ready() const;
    void wait_ready(double timeout_s) const;

private:
    using Clock = std::chrono::steady_clock;

    void mark_update_locked();
    static void read_joint_state(
        const std::shared_ptr<magic::z1::JointState>& msg,
        std::vector<float>& q,
        std::vector<float>& dq);

    mutable std::mutex mutex_;
    mutable std::condition_variable cv_;
    std::vector<float> leg_q_ = std::vector<float>(12, 0.0f);
    std::vector<float> leg_dq_ = std::vector<float>(12, 0.0f);
    std::vector<float> arm_q_ = std::vector<float>(14, 0.0f);
    std::vector<float> arm_dq_ = std::vector<float>(14, 0.0f);
    std::vector<float> waist_q_ = std::vector<float>(1, 0.0f);
    std::vector<float> waist_dq_ = std::vector<float>(1, 0.0f);
    std::vector<float> head_q_ = std::vector<float>(2, 0.0f);
    std::vector<float> head_dq_ = std::vector<float>(2, 0.0f);
    std::array<float, 4> quat_{1.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 3> ang_vel_{0.0f, 0.0f, 0.0f};
    Counts counts_{};
    bool have_imu_{false};
    Clock::time_point last_update_{};
};

class MagicbotSdkAdapter {
public:
    MagicbotSdkAdapter() = default;
    ~MagicbotSdkAdapter();

    void initialize_and_connect(const std::string& local_ip);
    void disconnect(bool skip_shutdown);
    void prepare_gait(const std::string& gait_name);
    void try_highlevel_passive(bool required);
    void play_tts(const std::string& text, bool required, const std::string& prompt_id);
    void enter_lowlevel(SdkRobotState& state);

    void publish_sdk24_command(
        const Counts& counts,
        const JointArray& target_motor,
        const JointArray& kp_motor,
        const JointArray& kd_motor,
        bool damping_only,
        float damping_kd);

    void publish_damping(const Counts& counts, float damping_kd);

private:
    magic::z1::GaitMode parse_gait(const std::string& gait_name) const;
    static magic::z1::JointCommand make_joint_command(int count);
    static void set_joint_command(
        magic::z1::SingleJointCommand& joint,
        float pos,
        float kp,
        float kd,
        int operation_mode);
    static std::pair<std::string, int> sdk24_group_index(int motor_idx, int arm_count);

    magic::z1::MagicRobot robot_;
    magic::z1::LowLevelMotionController* lowlevel_{nullptr};
    bool connected_{false};
    bool lowlevel_entered_{false};
};

}  // namespace magicbot_loco
