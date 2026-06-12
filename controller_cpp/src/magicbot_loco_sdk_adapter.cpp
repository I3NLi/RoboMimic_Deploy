#include "magicbot_loco_sdk_adapter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace magicbot_loco {
namespace {

constexpr int kHeadMotorIndex = 13;
constexpr float kHeadHoldTarget = 0.0f;

int64_t now_ns()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

void throw_status(const std::string& label, const magic::z1::Status& status)
{
    if (status.code == magic::z1::ErrorCode::OK) return;
    throw std::runtime_error(
        label + " failed: code=" + std::to_string(static_cast<int>(status.code)) +
        " message=" + status.message);
}

}  // namespace

void SdkRobotState::read_joint_state(
    const std::shared_ptr<magic::z1::JointState>& msg,
    std::vector<float>& q,
    std::vector<float>& dq)
{
    const size_t n = std::min(q.size(), msg ? msg->joints.size() : size_t{0});
    for (size_t i = 0; i < n; ++i) {
        q[i] = static_cast<float>(msg->joints[i].posL);
        const float vel = static_cast<float>(msg->joints[i].vel);
        dq[i] = (std::isfinite(vel) && std::fabs(vel) < 100.0f) ? vel : 0.0f;
    }
}

void SdkRobotState::mark_update_locked()
{
    last_update_ = Clock::now();
}

void SdkRobotState::update_leg(const std::shared_ptr<magic::z1::JointState> msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    read_joint_state(msg, leg_q_, leg_dq_);
    counts_.leg = static_cast<int>(msg ? msg->joints.size() : 0);
    mark_update_locked();
    cv_.notify_all();
}

void SdkRobotState::update_arm(const std::shared_ptr<magic::z1::JointState> msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    read_joint_state(msg, arm_q_, arm_dq_);
    counts_.arm = static_cast<int>(msg ? msg->joints.size() : 0);
    mark_update_locked();
    cv_.notify_all();
}

void SdkRobotState::update_waist(const std::shared_ptr<magic::z1::JointState> msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    read_joint_state(msg, waist_q_, waist_dq_);
    counts_.waist = static_cast<int>(msg ? msg->joints.size() : 0);
    mark_update_locked();
    cv_.notify_all();
}

void SdkRobotState::update_head(const std::shared_ptr<magic::z1::JointState> msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    read_joint_state(msg, head_q_, head_dq_);
    counts_.head = static_cast<int>(msg ? msg->joints.size() : 0);
    mark_update_locked();
    cv_.notify_all();
}

void SdkRobotState::update_imu(const std::shared_ptr<magic::z1::Imu> msg)
{
    if (!msg) return;
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < 4; ++i) quat_[i] = static_cast<float>(msg->orientation[i]);
    for (int i = 0; i < 3; ++i) ang_vel_[i] = static_cast<float>(msg->angular_velocity[i]);
    have_imu_ = true;
    mark_update_locked();
    cv_.notify_all();
}

RobotSnapshot SdkRobotState::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    RobotSnapshot out;
    for (int i = 0; i < 12; ++i) {
        out.q[i] = leg_q_[i];
        out.dq[i] = leg_dq_[i];
    }
    out.q[12] = waist_q_[0];
    out.dq[12] = waist_dq_[0];
    out.q[13] = head_q_[0];
    out.dq[13] = head_dq_[0];

    for (int i = 0; i < 5; ++i) {
        out.q[14 + i] = i < static_cast<int>(arm_q_.size()) ? arm_q_[i] : 0.0f;
        out.dq[14 + i] = i < static_cast<int>(arm_dq_.size()) ? arm_dq_[i] : 0.0f;
    }
    const int right_start = counts_.arm >= 12 ? 7 : 5;
    for (int i = 0; i < 5; ++i) {
        const int src = right_start + i;
        out.q[19 + i] = src < static_cast<int>(arm_q_.size()) ? arm_q_[src] : 0.0f;
        out.dq[19 + i] = src < static_cast<int>(arm_dq_.size()) ? arm_dq_[src] : 0.0f;
    }
    out.quat = quat_;
    out.ang_vel = ang_vel_;
    out.counts = counts_;
    return out;
}

double SdkRobotState::state_age_ms() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_update_ == Clock::time_point{}) return -1.0;
    return std::chrono::duration<double, std::milli>(Clock::now() - last_update_).count();
}

std::pair<bool, std::string> SdkRobotState::ready() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (counts_.leg < 12) return {false, "waiting leg state, got " + std::to_string(counts_.leg) + "/12"};
    if (counts_.arm < 10) return {false, "waiting arm state, got " + std::to_string(counts_.arm) + "/10"};
    if (counts_.waist < 1) return {false, "waiting waist state, got " + std::to_string(counts_.waist) + "/1"};
    if (counts_.head < 1) return {false, "waiting head state, got " + std::to_string(counts_.head) + "/1"};
    if (!have_imu_) return {false, "waiting body imu"};
    return {true, "ready"};
}

void SdkRobotState::wait_ready(double timeout_s) const
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    std::unique_lock<std::mutex> lock(mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
        const bool ok = counts_.leg >= 12 && counts_.arm >= 10 && counts_.waist >= 1 && counts_.head >= 1 && have_imu_;
        if (ok) return;
        cv_.wait_until(lock, std::chrono::steady_clock::now() + std::chrono::milliseconds(20));
    }
    lock.unlock();
    auto [ok, reason] = ready();
    if (!ok) throw std::runtime_error(reason);
}

MagicbotSdkAdapter::~MagicbotSdkAdapter()
{
    try {
        disconnect(false);
    } catch (...) {
    }
}

void MagicbotSdkAdapter::initialize_and_connect(const std::string& local_ip)
{
    std::cout << "[MagicBot] Initializing SDK with local_ip=" << local_ip << std::endl;
    if (!robot_.Initialize(local_ip)) {
        robot_.Shutdown();
        throw std::runtime_error("MagicBot SDK initialize failed for local_ip=" + local_ip);
    }
    throw_status("Connect", robot_.Connect());
    connected_ = true;
}

void MagicbotSdkAdapter::disconnect(bool skip_shutdown)
{
    if (lowlevel_ != nullptr) {
        try {
            lowlevel_->Shutdown();
        } catch (...) {
        }
        lowlevel_ = nullptr;
    }
    if (connected_ && !skip_shutdown) {
        try {
            (void)robot_.Disconnect();
        } catch (...) {
        }
    }
    if (!skip_shutdown) {
        try {
            robot_.Shutdown();
        } catch (...) {
        }
    }
    connected_ = false;
    lowlevel_entered_ = false;
}

magic::z1::GaitMode MagicbotSdkAdapter::parse_gait(const std::string& gait_name) const
{
    if (gait_name == "passive") return magic::z1::GaitMode::GAIT_PASSIVE;
    if (gait_name == "recovery" || gait_name == "recovery_stand") {
        return magic::z1::GaitMode::GAIT_RECOVERY_STAND;
    }
    throw std::runtime_error("unsupported gait: " + gait_name);
}

void MagicbotSdkAdapter::prepare_gait(const std::string& gait_name)
{
    if (gait_name.empty() || gait_name == "none" || gait_name == "skip") return;
    std::cout << "[MagicBot] Preparing gait before LowLevel: " << gait_name << std::endl;
    throw_status("SetMotionControlLevel(HighLevel)", robot_.SetMotionControlLevel(magic::z1::ControllerLevel::HighLevel));
    auto& high = robot_.GetHighLevelMotionController();
    if (!high.Initialize()) throw std::runtime_error("high-level motion controller initialize failed");
    throw_status("SetGait(" + gait_name + ")", high.SetGait(parse_gait(gait_name), 10000));
}

void MagicbotSdkAdapter::try_highlevel_passive(bool required)
{
    try {
        prepare_gait("passive");
        std::cout << "[MagicBot] HighLevel passive gait accepted" << std::endl;
    } catch (const std::exception& exc) {
        if (required) throw;
        std::cerr << "[MagicBot][WARN] HighLevel passive was not accepted: " << exc.what() << std::endl;
        std::cerr << "[MagicBot][WARN] Continuing with LowLevel damping-only passive fallback" << std::endl;
    }
}

void MagicbotSdkAdapter::play_tts(const std::string& text, bool required, const std::string& prompt_id)
{
    if (text.empty()) return;
    try {
        auto& audio = robot_.GetAudioController();
        (void)audio.Initialize();
        magic::z1::TtsCommand cmd;
        cmd.id = prompt_id.empty() ? "100000000001" : prompt_id;
        cmd.content = text;
        cmd.priority = magic::z1::TtsPriority::HIGH;
        cmd.mode = magic::z1::TtsMode::CLEARTOP;
        throw_status("TTS Play", audio.Play(cmd, 10000));
        std::cout << "[MagicBot] TTS prompt queued: " << text << std::endl;
    } catch (const std::exception& exc) {
        if (required) throw;
        std::cerr << "[MagicBot][WARN] TTS prompt failed: " << exc.what() << std::endl;
    }
}

void MagicbotSdkAdapter::enter_lowlevel(SdkRobotState& state)
{
    throw_status("SetMotionControlLevel(LowLevel)", robot_.SetMotionControlLevel(magic::z1::ControllerLevel::LowLevel));
    lowlevel_ = &robot_.GetLowLevelMotionController();
    std::cout << "[MagicBot] LowLevel controller initialize: " << (lowlevel_->Initialize() ? "true" : "false") << std::endl;
    lowlevel_->SubscribeLegState([&state](const std::shared_ptr<magic::z1::JointState> msg) { state.update_leg(msg); });
    lowlevel_->SubscribeArmState([&state](const std::shared_ptr<magic::z1::JointState> msg) { state.update_arm(msg); });
    lowlevel_->SubscribeWaistState([&state](const std::shared_ptr<magic::z1::JointState> msg) { state.update_waist(msg); });
    lowlevel_->SubscribeHeadState([&state](const std::shared_ptr<magic::z1::JointState> msg) { state.update_head(msg); });
    lowlevel_->SubscribeBodyImu([&state](const std::shared_ptr<magic::z1::Imu> msg) { state.update_imu(msg); });
    lowlevel_entered_ = true;
}

magic::z1::JointCommand MagicbotSdkAdapter::make_joint_command(int count)
{
    magic::z1::JointCommand command;
    command.timestamp = now_ns();
    command.joints.resize(static_cast<size_t>(std::max(0, count)));
    return command;
}

void MagicbotSdkAdapter::set_joint_command(
    magic::z1::SingleJointCommand& joint,
    float pos,
    float kp,
    float kd,
    int operation_mode)
{
    joint.operation_mode = static_cast<int16_t>(operation_mode);
    joint.pos = pos;
    joint.vel = 0.0;
    joint.toq = 0.0;
    joint.kp = kp;
    joint.kd = kd;
}

std::pair<std::string, int> MagicbotSdkAdapter::sdk24_group_index(int motor_idx, int arm_count)
{
    if (motor_idx < 12) return {"leg", motor_idx};
    if (motor_idx == 12) return {"waist", 0};
    if (motor_idx == 13) return {"head", 0};
    if (motor_idx >= 14 && motor_idx <= 18) return {"arm", motor_idx - 14};
    if (motor_idx >= 19 && motor_idx <= 23) return {"arm", (arm_count >= 12 ? 7 : 5) + (motor_idx - 19)};
    throw std::runtime_error("invalid SDK24 motor index");
}

void MagicbotSdkAdapter::publish_sdk24_command(
    const Counts& counts,
    const JointArray& target_motor,
    const JointArray& kp_motor,
    const JointArray& kd_motor,
    bool damping_only,
    float damping_kd)
{
    if (lowlevel_ == nullptr) throw std::runtime_error("LowLevel controller is not initialized");

    const int leg_count = std::max(12, std::min(counts.leg > 0 ? counts.leg : 12, static_cast<int>(magic::z1::kLegJointNum)));
    const int arm_count = counts.arm > 0 ? counts.arm : static_cast<int>(magic::z1::kArmJointNum);
    const int waist_count = counts.waist > 0 ? counts.waist : static_cast<int>(magic::z1::kWaistJointNum);
    const int head_count = counts.head > 0 ? counts.head : static_cast<int>(magic::z1::kHeadJointNum);

    auto leg_command = make_joint_command(leg_count);
    auto arm_command = make_joint_command(arm_count);
    auto waist_command = make_joint_command(waist_count);
    auto head_command = make_joint_command(head_count);

    auto fill_ready = [&](magic::z1::JointCommand& command) {
        for (auto& joint : command.joints) set_joint_command(joint, 0.0f, 0.0f, damping_only ? damping_kd : 0.0f, 200);
    };
    fill_ready(leg_command);
    fill_ready(arm_command);
    fill_ready(waist_command);
    fill_ready(head_command);

    for (int motor_idx = 0; motor_idx < kNumJoints; ++motor_idx) {
        const auto [group, group_idx] = sdk24_group_index(motor_idx, arm_count);
        if (group == "head") {
            if (group_idx >= 0 && group_idx < static_cast<int>(head_command.joints.size())) {
                if (damping_only) {
                    set_joint_command(
                        head_command.joints[static_cast<size_t>(group_idx)],
                        0.0f,
                        0.0f,
                        damping_kd,
                        200);
                } else {
                    set_joint_command(
                        head_command.joints[static_cast<size_t>(group_idx)],
                        kHeadHoldTarget,
                        kp_motor[kHeadMotorIndex],
                        kd_motor[kHeadMotorIndex],
                        200);
                }
            }
            continue;
        }

        magic::z1::JointCommand* command = nullptr;
        if (group == "leg") command = &leg_command;
        if (group == "arm") command = &arm_command;
        if (group == "waist") command = &waist_command;
        if (command == nullptr || group_idx < 0 || group_idx >= static_cast<int>(command->joints.size())) continue;

        if (damping_only) {
            set_joint_command(command->joints[static_cast<size_t>(group_idx)], 0.0f, 0.0f, damping_kd, 3);
        } else {
            set_joint_command(
                command->joints[static_cast<size_t>(group_idx)],
                target_motor[motor_idx],
                kp_motor[motor_idx],
                kd_motor[motor_idx],
                3);
        }
    }

    throw_status("PublishLegCommand", lowlevel_->PublishLegCommand(leg_command));
    throw_status("PublishArmCommand", lowlevel_->PublishArmCommand(arm_command));
    throw_status("PublishWaistCommand", lowlevel_->PublishWaistCommand(waist_command));
    throw_status("PublishHeadCommand", lowlevel_->PublishHeadCommand(head_command));
}

void MagicbotSdkAdapter::publish_damping(const Counts& counts, float damping_kd)
{
    JointArray zero{};
    publish_sdk24_command(counts, zero, zero, zero, true, damping_kd);
}

}  // namespace magicbot_loco
