/**
 * Headless MuJoCo DDS simulator.
 *
 * This is the C++/MuJoCo-API counterpart of the simulation transport part of
 * python_reference/simulation/mujoco_dds_compare.py:
 *   - load MJCF with MuJoCo C API
 *   - publish rt/lowstate from MuJoCo qpos/qvel/IMU state
 *   - subscribe rt/lowcmd from a policy process
 *   - apply q/kp/kd as PD torques to MuJoCo actuators
 *
 * It intentionally does not run policy inference and does not connect to the
 * real robot. Run it with robot_controller_onnx --shadow to exercise the C++ policy
 * against MuJoCo without Python in the loop.
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <time.h>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

namespace fs = std::filesystem;

namespace {

struct Args {
    std::string net = "lo";
    std::string mujoco_yaml = "configs/simulation/mujoco.yaml";
    std::string xml = "";
    std::string yaml = "policies/beyond_mimic/config/BeyondMimic.yaml";
    std::string initial_pose_yaml = "";
    std::string lowstate_topic = "rt/lowstate";
    std::string lowcmd_topic = "rt/lowcmd";
    int max_steps = 1000;          // control steps, not MuJoCo internal steps
    int print_every = 100;
    int control_decimation = 10;
    double sim_dt = 0.002;
    double wait_cmd_ms = 5.0;
    double damping_kd = 8.0;
    bool dry_run = false;
    bool realtime = true;
    bool state_lab_order = false;
    bool cmd_lab_order = false;
};

double now_sec()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

void sleep_sec(double s)
{
    if (s <= 0.0) return;
    struct timespec ts;
    ts.tv_sec = static_cast<time_t>(s);
    ts.tv_nsec = static_cast<long>((s - ts.tv_sec) * 1e9);
    nanosleep(&ts, nullptr);
}

void usage(const char* argv0)
{
    std::printf(
        "Usage: %s [--net IFACE] [--mujoco-yaml PATH] [--xml PATH] [--yaml PATH]\n"
        "          [--initial-pose-yaml PATH]\n"
        "          [--max-steps N] [--print-every N] [--dry-run]\n"
        "          [--no-realtime] [--state-lab-order] [--cmd-lab-order] [--dds-lab-order]\n",
        argv0);
}

std::string need_value(int& i, int argc, char** argv)
{
    if (i + 1 >= argc) {
        std::fprintf(stderr, "%s requires a value\n", argv[i]);
        std::exit(2);
    }
    return argv[++i];
}

Args parse_args(int argc, char** argv)
{
    Args args;
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--net") == 0) {
            args.net = need_value(i, argc, argv);
        } else if (std::strcmp(argv[i], "--mujoco-yaml") == 0) {
            args.mujoco_yaml = need_value(i, argc, argv);
        } else if (std::strcmp(argv[i], "--xml") == 0) {
            args.xml = need_value(i, argc, argv);
        } else if (std::strcmp(argv[i], "--yaml") == 0) {
            args.yaml = need_value(i, argc, argv);
        } else if (std::strcmp(argv[i], "--initial-pose-yaml") == 0) {
            args.initial_pose_yaml = need_value(i, argc, argv);
        } else if (std::strcmp(argv[i], "--lowstate-topic") == 0) {
            args.lowstate_topic = need_value(i, argc, argv);
        } else if (std::strcmp(argv[i], "--lowcmd-topic") == 0) {
            args.lowcmd_topic = need_value(i, argc, argv);
        } else if (std::strcmp(argv[i], "--max-steps") == 0) {
            args.max_steps = std::atoi(need_value(i, argc, argv).c_str());
        } else if (std::strcmp(argv[i], "--print-every") == 0) {
            args.print_every = std::atoi(need_value(i, argc, argv).c_str());
        } else if (std::strcmp(argv[i], "--control-decimation") == 0) {
            args.control_decimation = std::atoi(need_value(i, argc, argv).c_str());
        } else if (std::strcmp(argv[i], "--sim-dt") == 0) {
            args.sim_dt = std::atof(need_value(i, argc, argv).c_str());
        } else if (std::strcmp(argv[i], "--wait-cmd-ms") == 0) {
            args.wait_cmd_ms = std::atof(need_value(i, argc, argv).c_str());
        } else if (std::strcmp(argv[i], "--damping-kd") == 0) {
            args.damping_kd = std::atof(need_value(i, argc, argv).c_str());
        } else if (std::strcmp(argv[i], "--dry-run") == 0) {
            args.dry_run = true;
        } else if (std::strcmp(argv[i], "--no-realtime") == 0) {
            args.realtime = false;
        } else if (std::strcmp(argv[i], "--state-lab-order") == 0) {
            args.state_lab_order = true;
        } else if (std::strcmp(argv[i], "--cmd-lab-order") == 0) {
            args.cmd_lab_order = true;
        } else if (std::strcmp(argv[i], "--dds-lab-order") == 0) {
            args.state_lab_order = true;
            args.cmd_lab_order = true;
        } else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            std::exit(0);
        } else {
            std::fprintf(stderr, "Unknown arg: %s\n", argv[i]);
            usage(argv[0]);
            std::exit(2);
        }
    }
    args.max_steps = std::max(1, args.max_steps);
    args.print_every = std::max(1, args.print_every);
    args.control_decimation = std::max(1, args.control_decimation);
    return args;
}

fs::path repo_root()
{
    auto looks_like_root = [](const fs::path& p) {
        return fs::exists(p / "configs") &&
               fs::exists(p / "assets/robots/magicbot_z1") &&
               fs::exists(p / "policies");
    };

    for (fs::path p = fs::current_path(); !p.empty(); p = p.parent_path()) {
        if (looks_like_root(p)) return p;
        if (p == p.root_path()) break;
    }

    fs::path source_path = fs::absolute(fs::path(__FILE__));
    for (fs::path p = source_path.parent_path(); !p.empty(); p = p.parent_path()) {
        if (looks_like_root(p)) return p;
        if (p == p.root_path()) break;
    }

    return fs::current_path();
}

fs::path resolve_path(const fs::path& root, const std::string& raw)
{
    fs::path p(raw);
    if (p.is_absolute()) return p;
    return root / p;
}

void load_mujoco_yaml(Args& args, const fs::path& root)
{
    fs::path yaml_path = resolve_path(root, args.mujoco_yaml);
    if (!fs::exists(yaml_path)) return;
    YAML::Node cfg = YAML::LoadFile(yaml_path.string());
    if (args.xml.empty() && cfg["xml_path"]) {
        args.xml = cfg["xml_path"].as<std::string>();
    }
    if (args.initial_pose_yaml.empty() && cfg["initial_pose_yaml"]) {
        args.initial_pose_yaml = cfg["initial_pose_yaml"].as<std::string>();
    }
    if (cfg["simulation_dt"]) {
        args.sim_dt = cfg["simulation_dt"].as<double>();
    }
    if (cfg["control_decimation"]) {
        args.control_decimation = cfg["control_decimation"].as<int>();
    }
}

std::vector<int> load_mj2lab(const fs::path& root, const std::string& yaml_path_raw, int n)
{
    std::vector<int> mj2lab(n);
    for (int i = 0; i < n; i++) mj2lab[i] = i;
    if (yaml_path_raw.empty()) return mj2lab;
    fs::path yaml_path = resolve_path(root, yaml_path_raw);
    if (!fs::exists(yaml_path)) return mj2lab;
    YAML::Node cfg = YAML::LoadFile(yaml_path.string());
    if (!cfg["mj2lab"]) return mj2lab;
    auto raw = cfg["mj2lab"].as<std::vector<int>>();
    if ((int)raw.size() != n) {
        std::printf("[WARN] mj2lab length %zu != n=%d, using identity\n", raw.size(), n);
        return mj2lab;
    }
    return raw;
}

std::vector<int> actuator_qpos_indices(const mjModel* model)
{
    std::vector<int> idx(model->nu, 0);
    for (int actuator_id = 0; actuator_id < model->nu; actuator_id++) {
        int joint_id = model->actuator_trnid[2 * actuator_id + 0];
        idx[actuator_id] = model->jnt_qposadr[joint_id] - 7;
    }
    return idx;
}

std::vector<int> actuator_qvel_indices(const mjModel* model)
{
    std::vector<int> idx(model->nu, 0);
    for (int actuator_id = 0; actuator_id < model->nu; actuator_id++) {
        int joint_id = model->actuator_trnid[2 * actuator_id + 0];
        idx[actuator_id] = model->jnt_dofadr[joint_id] - 6;
    }
    return idx;
}

bool apply_initial_pose(
    const fs::path& root,
    const std::string& yaml_path_raw,
    const std::vector<int>& qpos_idx,
    const std::vector<int>& mj2lab,
    mjData* data)
{
    if (yaml_path_raw.empty()) return false;
    fs::path yaml_path = resolve_path(root, yaml_path_raw);
    if (!fs::exists(yaml_path)) return false;
    YAML::Node cfg = YAML::LoadFile(yaml_path.string());
    if (!cfg["default_angles_lab"]) return false;
    auto default_lab = cfg["default_angles_lab"].as<std::vector<double>>();
    const int n = static_cast<int>(qpos_idx.size());
    std::vector<double> default_mj(n, 0.0);
    for (int lab_i = 0; lab_i < (int)mj2lab.size() && lab_i < (int)default_lab.size(); lab_i++) {
        int mj_i = mj2lab[lab_i];
        if (mj_i >= 0 && mj_i < n) {
            default_mj[mj_i] = default_lab[lab_i];
        }
    }
    for (int actuator_i = 0; actuator_i < n; actuator_i++) {
        data->qpos[7 + qpos_idx[actuator_i]] = default_mj[actuator_i];
    }
    data->qvel[0] = 0.0;
    return true;
}

class DdsBridge {
public:
    DdsBridge(
        const Args& args,
        int num_joints,
        const std::vector<int>& mj2lab,
        const std::vector<int>& qpos_idx,
        const std::vector<int>& qvel_idx)
        : args_(args),
          num_joints_(num_joints),
          mj2lab_(mj2lab),
          qpos_idx_(qpos_idx),
          qvel_idx_(qvel_idx),
          state_pub_(args.lowstate_topic),
          cmd_sub_(args.lowcmd_topic)
    {
        target_q_.assign(num_joints_, 0.0);
        kp_.assign(num_joints_, 0.0);
        kd_.assign(num_joints_, 0.0);
        state_pub_.InitChannel();
        cmd_sub_.InitChannel([this](const void* msg) { this->on_lowcmd(msg); }, 10);
    }

    uint64_t recv_count() const { return recv_count_.load(); }

    void publish_state(const mjModel* m, const mjData* d)
    {
        (void)m;
        LowState_ state;
        state.tick() = tick_++;
        state.mode_machine() = 0;
        state.wireless_remote().fill(0);

        const int n = std::min<int>(num_joints_, state.motor_state().size());
        if (args_.state_lab_order) {
            for (int lab_i = 0; lab_i < n; lab_i++) {
                int mj_i = mj2lab_[lab_i];
                state.motor_state()[lab_i].q() = static_cast<float>(d->qpos[7 + qpos_idx_[mj_i]]);
                state.motor_state()[lab_i].dq() = static_cast<float>(d->qvel[6 + qvel_idx_[mj_i]]);
            }
        } else {
            for (int i = 0; i < n; i++) {
                state.motor_state()[i].q() = static_cast<float>(d->qpos[7 + qpos_idx_[i]]);
                state.motor_state()[i].dq() = static_cast<float>(d->qvel[6 + qvel_idx_[i]]);
            }
        }

        state.imu_state().quaternion()[0] = static_cast<float>(d->qpos[3]);
        state.imu_state().quaternion()[1] = static_cast<float>(d->qpos[4]);
        state.imu_state().quaternion()[2] = static_cast<float>(d->qpos[5]);
        state.imu_state().quaternion()[3] = static_cast<float>(d->qpos[6]);
        state.imu_state().gyroscope()[0] = static_cast<float>(d->qvel[3]);
        state.imu_state().gyroscope()[1] = static_cast<float>(d->qvel[4]);
        state.imu_state().gyroscope()[2] = static_cast<float>(d->qvel[5]);
        state_pub_.Write(state);
    }

    void snapshot_cmd(std::vector<double>& q, std::vector<double>& kp, std::vector<double>& kd)
    {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        q = target_q_;
        kp = kp_;
        kd = kd_;
    }

private:
    void on_lowcmd(const void* msg)
    {
        const auto* cmd = static_cast<const LowCmd_*>(msg);
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        const int n = std::min<int>(num_joints_, cmd->motor_cmd().size());
        if (args_.cmd_lab_order) {
            for (int lab_i = 0; lab_i < n; lab_i++) {
                int mj_i = mj2lab_[lab_i];
                target_q_[mj_i] = cmd->motor_cmd()[lab_i].q();
                kp_[mj_i] = cmd->motor_cmd()[lab_i].kp();
                kd_[mj_i] = cmd->motor_cmd()[lab_i].kd();
            }
        } else {
            for (int i = 0; i < n; i++) {
                target_q_[i] = cmd->motor_cmd()[i].q();
                kp_[i] = cmd->motor_cmd()[i].kp();
                kd_[i] = cmd->motor_cmd()[i].kd();
            }
        }
        recv_count_++;
    }

    Args args_;
    int num_joints_;
    std::vector<int> mj2lab_;
    std::vector<int> qpos_idx_;
    std::vector<int> qvel_idx_;
    ChannelPublisher<LowState_> state_pub_;
    ChannelSubscriber<LowCmd_> cmd_sub_;
    std::atomic<uint64_t> recv_count_{0};
    uint32_t tick_{1};
    std::mutex cmd_mtx_;
    std::vector<double> target_q_;
    std::vector<double> kp_;
    std::vector<double> kd_;
};

double finite_or(double v, double fallback)
{
    return std::isfinite(v) ? v : fallback;
}

void apply_pd_control(
    const Args& args,
    const mjModel* m,
    mjData* d,
    const std::vector<double>& target_q,
    const std::vector<double>& kp,
    const std::vector<double>& kd,
    const std::vector<int>& qpos_idx,
    const std::vector<int>& qvel_idx)
{
    for (int i = 0; i < m->nu; i++) {
        double q = d->qpos[7 + qpos_idx[i]];
        double dq = d->qvel[6 + qvel_idx[i]];
        double tau = (target_q[i] - q) * kp[i] - dq * kd[i];
        tau = finite_or(tau, -args.damping_kd * dq);
        if (m->actuator_ctrllimited && m->actuator_ctrllimited[i]) {
            double lo = m->actuator_ctrlrange[2 * i + 0];
            double hi = m->actuator_ctrlrange[2 * i + 1];
            tau = std::clamp(tau, lo, hi);
        } else {
            tau = std::clamp(tau, -300.0, 300.0);
        }
        d->ctrl[i] = args.dry_run ? 0.0 : tau;
    }
}

struct CommandSnapshot {
    uint64_t seq{0};
    double stamp{0.0};
    bool estop{false};
};

struct ActionSnapshot {
    uint64_t seq{0};
    uint64_t lowcmd_recv{0};
    double stamp{0.0};
    bool valid{false};
    std::vector<double> target_q;
    std::vector<double> kp;
    std::vector<double> kd;
};

struct ViewSnapshot {
    int control_step{0};
    uint64_t lowcmd_recv{0};
    double q0{0.0};
    double dq0{0.0};
    double hz{0.0};
    bool done{false};
};

template <typename T>
class SharedValue {
public:
    SharedValue() = default;
    explicit SharedValue(T value) : value_(std::move(value)) {}

    void store(const T& value)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        value_ = value;
    }

    T load() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return value_;
    }

private:
    mutable std::mutex mtx_;
    T value_{};
};

struct RuntimeBuffers {
    std::atomic<bool> stop{false};
    SharedValue<CommandSnapshot> command;
    SharedValue<ActionSnapshot> action;
    SharedValue<ViewSnapshot> view;
};

void input_thread_fn(RuntimeBuffers& buffers, double period_s)
{
    uint64_t seq = 0;
    while (!buffers.stop.load()) {
        CommandSnapshot cmd;
        cmd.seq = ++seq;
        cmd.stamp = now_sec();
        cmd.estop = false;
        buffers.command.store(cmd);
        sleep_sec(period_s);
    }
}

void inference_thread_fn(
    RuntimeBuffers& buffers,
    DdsBridge& bridge,
    int num_joints)
{
    uint64_t last_recv = 0;
    uint64_t seq = 0;
    std::vector<double> target_q(num_joints, 0.0);
    std::vector<double> kp(num_joints, 0.0);
    std::vector<double> kd(num_joints, 0.0);

    while (!buffers.stop.load()) {
        uint64_t recv = bridge.recv_count();
        if (recv != last_recv) {
            bridge.snapshot_cmd(target_q, kp, kd);
            ActionSnapshot action;
            action.seq = ++seq;
            action.lowcmd_recv = recv;
            action.stamp = now_sec();
            action.valid = recv > 0;
            action.target_q = target_q;
            action.kp = kp;
            action.kd = kd;
            buffers.action.store(action);
            last_recv = recv;
        } else {
            sleep_sec(0.0005);
        }
    }
}

void view_thread_fn(const Args& args, RuntimeBuffers& buffers)
{
    int last_printed = -1;
    while (!buffers.stop.load()) {
        ViewSnapshot view = buffers.view.load();
        if (view.done) break;
        if (view.control_step != last_printed &&
            (view.control_step % args.print_every) == 0) {
            std::printf(
                "[view step=%d] lowcmd_recv=%llu q0=%.5f dq0=%.5f hz=%.2f\n",
                view.control_step,
                static_cast<unsigned long long>(view.lowcmd_recv),
                view.q0,
                view.dq0,
                view.hz);
            last_printed = view.control_step;
        }
        sleep_sec(0.01);
    }
}

void control_thread_fn(
    const Args& args,
    mjModel* model,
    mjData* data,
    DdsBridge& bridge,
    RuntimeBuffers& buffers,
    const std::vector<int>& qpos_idx,
    const std::vector<int>& qvel_idx)
{
    std::vector<double> target_q(model->nu, 0.0);
    std::vector<double> kp(model->nu, 0.0);
    std::vector<double> kd(model->nu, 0.0);

    int control_step = 0;
    double t0 = now_sec();
    while (!buffers.stop.load() && control_step < args.max_steps) {
        double step_start = now_sec();
        CommandSnapshot command = buffers.command.load();
        ActionSnapshot action = buffers.action.load();
        if (action.valid &&
            action.target_q.size() == static_cast<size_t>(model->nu) &&
            action.kp.size() == static_cast<size_t>(model->nu) &&
            action.kd.size() == static_cast<size_t>(model->nu)) {
            target_q = action.target_q;
            kp = action.kp;
            kd = action.kd;
        }
        Args control_args = args;
        if (command.estop) control_args.dry_run = true;

        for (int i = 0; i < args.control_decimation; i++) {
            apply_pd_control(control_args, model, data, target_q, kp, kd, qpos_idx, qvel_idx);
            mj_step(model, data);
        }

        bridge.publish_state(model, data);
        if (args.wait_cmd_ms > 0.0) {
            double deadline = now_sec() + args.wait_cmd_ms * 1e-3;
            while (bridge.recv_count() <= action.lowcmd_recv && now_sec() < deadline) {
                sleep_sec(0.0002);
            }
        }

        double elapsed = now_sec() - t0;
        ViewSnapshot view;
        view.control_step = control_step;
        view.lowcmd_recv = bridge.recv_count();
        view.q0 = data->qpos[7 + qpos_idx[0]];
        view.dq0 = data->qvel[6 + qvel_idx[0]];
        view.hz = (control_step + 1) / std::max(elapsed, 1e-9);
        buffers.view.store(view);

        if (args.realtime) {
            double control_dt = args.sim_dt * args.control_decimation;
            double remain = control_dt - (now_sec() - step_start);
            if (remain > 0.0) sleep_sec(remain);
        }
        control_step++;
    }

    double elapsed = now_sec() - t0;
    ViewSnapshot done = buffers.view.load();
    done.control_step = control_step;
    done.lowcmd_recv = bridge.recv_count();
    done.hz = control_step / std::max(elapsed, 1e-9);
    done.done = true;
    buffers.view.store(done);
    buffers.stop.store(true);
}

}  // namespace

int main(int argc, char** argv)
{
    Args args = parse_args(argc, argv);
    fs::path root = repo_root();
    load_mujoco_yaml(args, root);
    if (args.xml.empty()) {
        args.xml = "assets/robots/magicbot_z1/scene.xml";
    }
    fs::path xml_path = resolve_path(root, args.xml);

    char error[1024] = {0};
    mjModel* model = mj_loadXML(xml_path.string().c_str(), nullptr, error, sizeof(error));
    if (!model) {
        std::fprintf(stderr, "mj_loadXML failed: %s\n", error);
        return 1;
    }
    mjData* data = mj_makeData(model);
    model->opt.timestep = args.sim_dt;

    const int num_joints = model->nu;
    if (model->nq < 7 + num_joints || model->nv < 6 + num_joints) {
        std::fprintf(stderr, "Unsupported model dimensions nq=%d nv=%d nu=%d\n", model->nq, model->nv, model->nu);
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }
    std::vector<int> mj2lab = load_mj2lab(root, args.yaml, num_joints);
    std::vector<int> qpos_idx = actuator_qpos_indices(model);
    std::vector<int> qvel_idx = actuator_qvel_indices(model);
    bool initialized = apply_initial_pose(root, args.initial_pose_yaml, qpos_idx, mj2lab, data);

    ChannelFactory::Instance()->Init(0, args.net);
    DdsBridge bridge(args, num_joints, mj2lab, qpos_idx, qvel_idx);

    std::printf("=== MuJoCo DDS Sim (C++ MuJoCo API) ===\n");
    std::printf("XML      : %s\n", xml_path.string().c_str());
    std::printf("NET      : %s\n", args.net.c_str());
    std::printf("Joints   : %d\n", num_joints);
    std::printf("sim_dt   : %.6f\n", args.sim_dt);
    std::printf("ctrl_dt  : %.6f\n", args.sim_dt * args.control_decimation);
    std::printf("LowState : %s\n", args.lowstate_topic.c_str());
    std::printf("LowCmd   : %s\n", args.lowcmd_topic.c_str());
    std::printf("Realtime : %s\n", args.realtime ? "yes" : "no");
    std::printf("InitPose : %s\n", initialized ? args.initial_pose_yaml.c_str() : "(none)");
    std::printf("Order    : state=%s cmd=%s\n",
                args.state_lab_order ? "lab" : "mujoco",
                args.cmd_lab_order ? "lab" : "mujoco");

    RuntimeBuffers buffers;
    ActionSnapshot initial_action;
    initial_action.target_q.assign(num_joints, 0.0);
    initial_action.kp.assign(num_joints, 0.0);
    initial_action.kd.assign(num_joints, 0.0);
    buffers.action.store(initial_action);
    ViewSnapshot initial_view;
    initial_view.q0 = data->qpos[7 + qpos_idx[0]];
    initial_view.dq0 = data->qvel[6 + qvel_idx[0]];
    buffers.view.store(initial_view);

    std::thread input_thread(input_thread_fn, std::ref(buffers), args.sim_dt * args.control_decimation);
    std::thread inference_thread(inference_thread_fn, std::ref(buffers), std::ref(bridge), num_joints);
    std::thread view_thread(view_thread_fn, std::cref(args), std::ref(buffers));
    std::thread control_thread(
        control_thread_fn,
        std::cref(args),
        model,
        data,
        std::ref(bridge),
        std::ref(buffers),
        std::cref(qpos_idx),
        std::cref(qvel_idx));

    control_thread.join();
    buffers.stop.store(true);
    input_thread.join();
    inference_thread.join();
    view_thread.join();

    ViewSnapshot summary = buffers.view.load();
    std::printf("[Summary] control_steps=%d lowcmd_recv=%llu hz=%.2f\n",
                summary.control_step,
                static_cast<unsigned long long>(summary.lowcmd_recv),
                summary.hz);

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
