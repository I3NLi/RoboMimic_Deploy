#pragma once

#include "robot_adapter.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace magicbot_loco {

struct MujocoSimAdapterOptions {
    std::vector<int> qpos_idx;
    std::vector<int> qvel_idx;
    bool zero_head_target{false};
};

class MujocoSimAdapter final : public RobotAdapter {
public:
    MujocoSimAdapter(mjModel* model, mjData* data, MujocoSimAdapterOptions options)
        : model_(model),
          data_(data),
          options_(std::move(options))
    {
        if (model_ == nullptr || data_ == nullptr) {
            throw std::runtime_error("MujocoSimAdapter requires non-null mjModel and mjData");
        }
        if (options_.qpos_idx.size() != kNumJoints || options_.qvel_idx.size() != kNumJoints) {
            throw std::runtime_error("MujocoSimAdapter requires 24 qpos and qvel indices");
        }
    }

    const char* name() const override { return "mujoco-sim"; }

    RobotSnapshot read_snapshot() override
    {
        RobotSnapshot snap;
        for (int i = 0; i < kNumJoints; ++i) {
            snap.q[i] = static_cast<float>(data_->qpos[7 + options_.qpos_idx[static_cast<size_t>(i)]]);
            snap.dq[i] = static_cast<float>(data_->qvel[6 + options_.qvel_idx[static_cast<size_t>(i)]]);
        }
        if (model_->nq >= 7) {
            snap.quat = {
                static_cast<float>(data_->qpos[3]),
                static_cast<float>(data_->qpos[4]),
                static_cast<float>(data_->qpos[5]),
                static_cast<float>(data_->qpos[6]),
            };
        }
        if (model_->nv >= 6) {
            snap.ang_vel = {
                static_cast<float>(data_->qvel[3]),
                static_cast<float>(data_->qvel[4]),
                static_cast<float>(data_->qvel[5]),
            };
        }
        snap.counts = Counts{12, 14, 1, 2};
        return snap;
    }

    void write_target(const JointTarget& target) override
    {
        if (target.damping_only) {
            write_damping(target.damping_kd);
            return;
        }
        for (int i = 0; i < kNumJoints && i < model_->nu; ++i) {
            const double q = data_->qpos[7 + options_.qpos_idx[static_cast<size_t>(i)]];
            const double dq = data_->qvel[6 + options_.qvel_idx[static_cast<size_t>(i)]];
            const double target_q =
                (options_.zero_head_target && i == kHeadMotorIndex) ? 0.0 : static_cast<double>(target.q[i]);
            double tau = (target_q - q) * target.gains.kp[i] - dq * target.gains.kd[i];
            clamp_tau_limit(i, target.gains.tau_limit, tau);
            clamp_actuator(i, tau);
            data_->ctrl[i] = tau;
        }
        command_published_ = true;
    }

    void write_damping(float damping_kd) override
    {
        for (int i = 0; i < kNumJoints && i < model_->nu; ++i) {
            const double dq = data_->qvel[6 + options_.qvel_idx[static_cast<size_t>(i)]];
            double tau = -dq * damping_kd;
            clamp_actuator(i, tau);
            data_->ctrl[i] = tau;
        }
        command_published_ = true;
    }

    AdapterTelemetry telemetry() const override
    {
        AdapterTelemetry out;
        out.backend = name();
        out.command_published = command_published_;
        return out;
    }

private:
    static constexpr int kHeadMotorIndex = 13;

    void clamp_tau_limit(int joint_id, const JointArray& tau_limit, double& tau) const
    {
        const double limit = std::max(0.0f, tau_limit[static_cast<size_t>(joint_id)]);
        if (limit > 0.0) tau = std::clamp(tau, -limit, limit);
    }

    void clamp_actuator(int actuator_id, double& tau) const
    {
        if (model_->actuator_ctrllimited && model_->actuator_ctrllimited[actuator_id]) {
            const double lo = model_->actuator_ctrlrange[2 * actuator_id + 0];
            const double hi = model_->actuator_ctrlrange[2 * actuator_id + 1];
            tau = std::clamp(tau, lo, hi);
        }
    }

    mjModel* model_{nullptr};
    mjData* data_{nullptr};
    MujocoSimAdapterOptions options_{};
    bool command_published_{false};
};

}  // namespace magicbot_loco
