/**
 * Native dual inference rate tool for MagicBot Z1.
 *
 * Modes:
 *   - pure-sim: run MuJoCo dynamics and ONNX loco inference from simulated state.
 *   - real-state-sim: subscribe LowLevel state, feed it through ONNX loco, and
 *     optionally advance a local MuJoCo model for load measurement.
 *
 * The tool never publishes joint commands.
 */

#include "controller_core.h"
#include "controller_runtime.h"
#include "magicbot_loco_core.h"
#include "mujoco_sim_adapter.h"

#ifdef ENABLE_MAGICBOT_SDK
#include "magicbot_loco_sdk_adapter.h"
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
namespace ml = magicbot_loco;

namespace {

constexpr int kHeadMotorIndex = 13;

std::atomic<bool> g_running{true};

void signal_handler(int signum)
{
    std::cerr << "[Signal] " << signum << ", stopping" << std::endl;
    g_running.store(false);
}

struct Args {
    std::string mode{"pure-sim"};
    fs::path config{"policies/loco_mode/config/LocoMode_lowKp.yaml"};
    fs::path mujoco_yaml{"configs/simulation/mujoco.yaml"};
    std::string xml;
    std::string initial_pose_yaml;
    double duration{10.0};
    double sim_dt{0.0};
    int control_decimation{0};
    bool realtime{true};
    bool real_forward_only{false};
    fs::path summary_json;
    std::string local_ip;
    bool skip_network_check{false};
    double state_timeout{10.0};
    bool real_disconnect{false};
    float vx{0.0f};
    float vy{0.0f};
    float wz{0.0f};
    double damping_kd{8.0};
    bool ground_correction{false};
    std::string ground_floor_geom{"floor"};
    std::vector<std::string> ground_body_keywords;
    double ground_max_penetration{0.0};
    std::string push_body{"pelvis"};
    std::array<double, 3> push_force{0.0, 0.0, 0.0};
    std::array<double, 3> push_impulse{0.0, 0.0, 0.0};
    double push_start_s{0.0};
    double push_duration_s{0.0};
    double push_impulse_time_s{-1.0};
    bool closed_loop_check{false};
    double min_control_hz{0.0};
    double max_deadline_miss_ratio{-1.0};
    double max_infer_p99_ms{0.0};
    double min_base_height{0.0};
    double max_gravity_xy{0.0};
    double max_root_xy_drift{0.0};
    double max_abs_dq_limit{0.0};
    double max_abs_tau_limit{0.0};
};

struct SimContext {
    mjModel* model{nullptr};
    mjData* data{nullptr};
    std::vector<int> qpos_idx;
    std::vector<int> qvel_idx;
    int root_body_id{-1};
    int push_body_id{-1};
    int floor_geom_id{-1};
    std::vector<int> ground_contact_geom_ids;
    ml::JointArray init_q{};
    bool have_init_q{false};
};

struct Summary {
    std::string mode;
    bool real_forward_only{false};
    double duration_s{0.0};
    double elapsed_s{0.0};
    int sim_steps{0};
    int control_steps{0};
    double sim_hz{0.0};
    double control_hz{0.0};
    double target_sim_hz{0.0};
    double target_control_hz{0.0};
    double simulation_dt{0.0};
    int control_decimation{0};
    int deadline_misses{0};
    double infer_mean_ms{0.0};
    double infer_p95_ms{0.0};
    double infer_p99_ms{0.0};
    double infer_max_ms{0.0};
    double state_age_mean_ms{-1.0};
    double state_age_max_ms{-1.0};
    double max_abs_tau{0.0};
    double max_abs_q{0.0};
    double max_abs_dq{0.0};
    double min_base_height{0.0};
    double max_gravity_xy{0.0};
    double max_root_xy_drift{0.0};
    double max_policy_target_jump{0.0};
    std::string push_body;
    bool push_enabled{false};
    double push_start_s{0.0};
    double push_duration_s{0.0};
    double push_impulse_time_s{0.0};
    double push_force_norm{0.0};
    double push_impulse_norm{0.0};
    int push_force_steps{0};
    bool push_impulse_applied{false};
    bool pass{true};
    std::string fail_reason;
};

double now_sec()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
}

void sleep_sec(double seconds)
{
    if (seconds <= 0.0) return;
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

std::array<double, 3> parse_vec3(std::string value)
{
    std::replace(value.begin(), value.end(), ',', ' ');
    std::replace(value.begin(), value.end(), ';', ' ');
    std::istringstream iss(value);
    std::array<double, 3> out{0.0, 0.0, 0.0};
    if (!(iss >> out[0] >> out[1] >> out[2])) {
        throw std::runtime_error("expected vec3 formatted as x,y,z");
    }
    std::string extra;
    if (iss >> extra) {
        throw std::runtime_error("expected exactly three vec3 components");
    }
    for (double v : out) {
        if (!std::isfinite(v)) throw std::runtime_error("vec3 components must be finite");
    }
    return out;
}

double vec3_norm(const std::array<double, 3>& value)
{
    return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

bool has_vec3(const std::array<double, 3>& value)
{
    return vec3_norm(value) > 0.0;
}

void usage(const char* argv0)
{
    std::cout
        << "Usage: " << argv0 << " --mode pure-sim|real-state-sim [options]\n"
        << "\n"
        << "Native rate/load test. It runs ONNX loco inference and MuJoCo locally,\n"
        << "and never publishes joint commands.\n"
        << "\n"
        << "Options:\n"
        << "  --config PATH              Loco YAML config\n"
        << "  --mujoco-yaml PATH         MuJoCo runtime YAML\n"
        << "  --xml PATH                 Override MJCF path\n"
        << "  --initial-pose-yaml PATH   Override initial pose YAML\n"
        << "  --duration S               Test duration\n"
        << "  --sim-dt S                 Override MuJoCo timestep\n"
        << "  --control-decimation N     Override policy decimation\n"
        << "  --no-realtime              Run as fast as possible\n"
        << "  --real-forward-only        In real-state-sim, kinematic mj_forward only\n"
        << "  --summary-json PATH        Write RATE_SUMMARY JSON\n"
        << "  --local-ip IP              SDK local IP for real-state-sim\n"
        << "  --skip-network-check       Skip local IP preflight\n"
        << "  --real-disconnect          Run full SDK Disconnect/Shutdown on exit\n"
        << "  --vx V --vy V --wz V       Normalized command inputs; YAML cmd_range maps physical speed\n"
        << "\n"
        << "Disturbance test, pure-sim/forward sim only:\n"
        << "  --push-body NAME           Body receiving external force, default pelvis\n"
        << "  --push-force X,Y,Z         Continuous world-frame force in Newtons\n"
        << "  --push-start S             Force start time in sim seconds\n"
        << "  --push-duration S          Force duration in seconds\n"
        << "  --push-impulse X,Y,Z       One-step world-frame impulse in N*s\n"
        << "  --push-impulse-time S      Impulse time in sim seconds, default --push-start\n"
        << "\n"
        << "Closed-loop acceptance:\n"
        << "  --closed-loop-check        Enable pass/fail checks with conservative defaults\n"
        << "  --min-control-hz HZ        Require measured control Hz\n"
        << "  --max-deadline-miss-ratio R Require deadline misses / steps <= R\n"
        << "  --max-infer-p99-ms MS      Require ONNX p99 latency <= MS\n"
        << "  --min-base-height M        Require simulated base height >= M\n"
        << "  --max-gravity-xy V         Require max projected gravity xy <= V\n"
        << "  --max-root-xy-drift M      Require root xy drift <= M\n"
        << "  --max-abs-dq V             Require max joint velocity <= V\n"
        << "  --max-abs-tau T            Require max actuator torque <= T\n";
}

std::string need_value(int& i, int argc, char** argv)
{
    if (i + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + argv[i]);
    }
    return argv[++i];
}

Args parse_args(int argc, char** argv)
{
    Args args;
    const char* env_ip = std::getenv("MAGICBOT_LOCAL_IP");
    args.local_ip = env_ip && *env_ip ? env_ip : "192.168.54.119";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            usage(argv[0]);
            std::exit(0);
        } else if (a == "--mode") {
            args.mode = need_value(i, argc, argv);
        } else if (a == "--config") {
            args.config = need_value(i, argc, argv);
        } else if (a == "--mujoco-yaml") {
            args.mujoco_yaml = need_value(i, argc, argv);
        } else if (a == "--xml") {
            args.xml = need_value(i, argc, argv);
        } else if (a == "--initial-pose-yaml") {
            args.initial_pose_yaml = need_value(i, argc, argv);
        } else if (a == "--duration") {
            args.duration = std::stod(need_value(i, argc, argv));
        } else if (a == "--sim-dt") {
            args.sim_dt = std::stod(need_value(i, argc, argv));
        } else if (a == "--control-decimation") {
            args.control_decimation = std::stoi(need_value(i, argc, argv));
        } else if (a == "--no-realtime") {
            args.realtime = false;
        } else if (a == "--real-forward-only") {
            args.real_forward_only = true;
        } else if (a == "--summary-json") {
            args.summary_json = need_value(i, argc, argv);
        } else if (a == "--local-ip") {
            args.local_ip = need_value(i, argc, argv);
        } else if (a == "--skip-network-check") {
            args.skip_network_check = true;
        } else if (a == "--state-timeout") {
            args.state_timeout = std::stod(need_value(i, argc, argv));
        } else if (a == "--real-disconnect") {
            args.real_disconnect = true;
        } else if (a == "--vx") {
            args.vx = std::stof(need_value(i, argc, argv));
        } else if (a == "--vy") {
            args.vy = std::stof(need_value(i, argc, argv));
        } else if (a == "--wz") {
            args.wz = std::stof(need_value(i, argc, argv));
        } else if (a == "--damping-kd") {
            args.damping_kd = std::stod(need_value(i, argc, argv));
        } else if (a == "--push-body") {
            args.push_body = need_value(i, argc, argv);
        } else if (a == "--push-force") {
            args.push_force = parse_vec3(need_value(i, argc, argv));
        } else if (a == "--push-start") {
            args.push_start_s = std::stod(need_value(i, argc, argv));
        } else if (a == "--push-duration") {
            args.push_duration_s = std::stod(need_value(i, argc, argv));
        } else if (a == "--push-impulse") {
            args.push_impulse = parse_vec3(need_value(i, argc, argv));
        } else if (a == "--push-impulse-time") {
            args.push_impulse_time_s = std::stod(need_value(i, argc, argv));
        } else if (a == "--closed-loop-check") {
            args.closed_loop_check = true;
        } else if (a == "--min-control-hz") {
            args.min_control_hz = std::stod(need_value(i, argc, argv));
        } else if (a == "--max-deadline-miss-ratio") {
            args.max_deadline_miss_ratio = std::stod(need_value(i, argc, argv));
        } else if (a == "--max-infer-p99-ms") {
            args.max_infer_p99_ms = std::stod(need_value(i, argc, argv));
        } else if (a == "--min-base-height") {
            args.min_base_height = std::stod(need_value(i, argc, argv));
        } else if (a == "--max-gravity-xy") {
            args.max_gravity_xy = std::stod(need_value(i, argc, argv));
        } else if (a == "--max-root-xy-drift") {
            args.max_root_xy_drift = std::stod(need_value(i, argc, argv));
        } else if (a == "--max-abs-dq") {
            args.max_abs_dq_limit = std::stod(need_value(i, argc, argv));
        } else if (a == "--max-abs-tau") {
            args.max_abs_tau_limit = std::stod(need_value(i, argc, argv));
        } else {
            throw std::runtime_error("unknown argument: " + a);
        }
    }

    if (args.mode != "pure-sim" && args.mode != "real-state-sim") {
        throw std::runtime_error("--mode must be pure-sim or real-state-sim");
    }
    args.duration = std::max(0.001, args.duration);
    args.push_start_s = std::max(0.0, args.push_start_s);
    args.push_duration_s = std::max(0.0, args.push_duration_s);
    if (args.push_impulse_time_s < 0.0) args.push_impulse_time_s = args.push_start_s;
    if (has_vec3(args.push_force) && args.push_duration_s <= 0.0) {
        throw std::runtime_error("--push-duration must be > 0 when --push-force is nonzero");
    }
    if ((has_vec3(args.push_force) || has_vec3(args.push_impulse)) && args.mode == "real-state-sim" &&
        args.real_forward_only) {
        throw std::runtime_error("--push-* requires a dynamic MuJoCo step; remove --real-forward-only");
    }
    if (args.closed_loop_check) {
        if (args.max_deadline_miss_ratio < 0.0) args.max_deadline_miss_ratio = args.realtime ? 0.05 : 1.0;
        if (args.max_infer_p99_ms <= 0.0) args.max_infer_p99_ms = 2.0;
        if (args.min_base_height <= 0.0 && args.mode == "pure-sim") args.min_base_height = 0.35;
        if (args.max_gravity_xy <= 0.0) args.max_gravity_xy = 0.9;
        if (args.max_abs_dq_limit <= 0.0) args.max_abs_dq_limit = 60.0;
    }
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

fs::path resolve_path(const fs::path& root, const fs::path& raw)
{
    if (raw.is_absolute()) return raw;
    return root / raw;
}

std::vector<int> actuator_qpos_indices(const mjModel* model)
{
    std::vector<int> idx(model->nu, 0);
    for (int actuator_id = 0; actuator_id < model->nu; ++actuator_id) {
        const int joint_id = model->actuator_trnid[2 * actuator_id + 0];
        idx[actuator_id] = model->jnt_qposadr[joint_id] - 7;
    }
    return idx;
}

std::vector<int> actuator_qvel_indices(const mjModel* model)
{
    std::vector<int> idx(model->nu, 0);
    for (int actuator_id = 0; actuator_id < model->nu; ++actuator_id) {
        const int joint_id = model->actuator_trnid[2 * actuator_id + 0];
        idx[actuator_id] = model->jnt_dofadr[joint_id] - 6;
    }
    return idx;
}

std::vector<int> load_mj2lab(const fs::path& yaml_path, int n)
{
    std::vector<int> mj2lab(n);
    for (int i = 0; i < n; ++i) mj2lab[i] = i;
    if (!fs::exists(yaml_path)) return mj2lab;
    YAML::Node cfg = YAML::LoadFile(yaml_path.string());
    if (!cfg["mj2lab"]) return mj2lab;
    auto raw = cfg["mj2lab"].as<std::vector<int>>();
    if (static_cast<int>(raw.size()) != n) return mj2lab;
    return raw;
}

bool load_initial_pose(
    const fs::path& yaml_path,
    const std::vector<int>& mj2lab,
    ml::JointArray& init_q)
{
    if (!fs::exists(yaml_path)) return false;
    YAML::Node cfg = YAML::LoadFile(yaml_path.string());
    if (!cfg["default_angles_lab"]) return false;
    auto default_lab = cfg["default_angles_lab"].as<std::vector<float>>();
    init_q.fill(0.0f);
    for (int lab_i = 0; lab_i < static_cast<int>(default_lab.size()) &&
                        lab_i < static_cast<int>(mj2lab.size()); ++lab_i) {
        const int mj_i = mj2lab[lab_i];
        if (mj_i >= 0 && mj_i < ml::kNumJoints) init_q[mj_i] = default_lab[lab_i];
    }
    return true;
}

void load_mujoco_yaml(Args& args, const fs::path& root, double& yaml_sim_dt, int& yaml_decimation, double& base_height)
{
    const fs::path path = resolve_path(root, args.mujoco_yaml);
    if (!fs::exists(path)) return;
    YAML::Node cfg = YAML::LoadFile(path.string());
    if (args.xml.empty() && cfg["xml_path"]) args.xml = cfg["xml_path"].as<std::string>();
    if (args.initial_pose_yaml.empty() && cfg["initial_pose_yaml"]) {
        args.initial_pose_yaml = cfg["initial_pose_yaml"].as<std::string>();
    }
    if (cfg["simulation_dt"]) yaml_sim_dt = cfg["simulation_dt"].as<double>();
    if (cfg["control_decimation"]) yaml_decimation = cfg["control_decimation"].as<int>();
    if (cfg["initial_base_height"]) base_height = cfg["initial_base_height"].as<double>();
    if (cfg["ground_penetration_correction"]) {
        YAML::Node ground = cfg["ground_penetration_correction"];
        if (ground["enable"]) args.ground_correction = ground["enable"].as<bool>();
        if (ground["floor_geom"]) args.ground_floor_geom = ground["floor_geom"].as<std::string>();
        if (ground["max_penetration"]) args.ground_max_penetration = ground["max_penetration"].as<double>();
        if (ground["body_keywords"]) {
            args.ground_body_keywords.clear();
            for (const auto& keyword : ground["body_keywords"]) {
                args.ground_body_keywords.push_back(keyword.as<std::string>());
            }
        }
    }
}

bool int_vec_contains(const std::vector<int>& values, int value)
{
    return std::find(values.begin(), values.end(), value) != values.end();
}

std::vector<int> resolve_contact_geom_ids(const mjModel* model, const std::vector<std::string>& body_keywords)
{
    std::vector<int> geom_ids;
    for (int geom_id = 0; geom_id < model->ngeom; ++geom_id) {
        if (body_keywords.empty()) {
            geom_ids.push_back(geom_id);
            continue;
        }
        const int body_id = model->geom_bodyid[geom_id];
        const char* raw_name = mj_id2name(model, mjOBJ_BODY, body_id);
        const std::string body_name = raw_name ? raw_name : "";
        for (const auto& keyword : body_keywords) {
            if (!keyword.empty() && body_name.find(keyword) != std::string::npos) {
                geom_ids.push_back(geom_id);
                break;
            }
        }
    }
    return geom_ids;
}

double correct_ground_penetration(
    mjModel* model,
    mjData* data,
    int floor_geom_id,
    const std::vector<int>& contact_geom_ids,
    double max_penetration)
{
    if (floor_geom_id < 0 || max_penetration < 0.0 || contact_geom_ids.empty()) return 0.0;

    double min_dist = 0.0;
    for (int contact_id = 0; contact_id < data->ncon; ++contact_id) {
        const mjContact& contact = data->contact[contact_id];
        const int geom1 = contact.geom1;
        const int geom2 = contact.geom2;
        if ((geom1 == floor_geom_id && int_vec_contains(contact_geom_ids, geom2)) ||
            (geom2 == floor_geom_id && int_vec_contains(contact_geom_ids, geom1))) {
            min_dist = std::min(min_dist, static_cast<double>(contact.dist));
        }
    }

    const double allowed_dist = -max_penetration;
    if (min_dist >= allowed_dist) return 0.0;

    const double correction = allowed_dist - min_dist;
    data->qpos[2] += correction;
    if (model->nv > 2 && data->qvel[2] < 0.0) data->qvel[2] = 0.0;
    mj_forward(model, data);
    return correction;
}

double percentile(std::vector<double> values, double q)
{
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const auto idx = std::min(
        values.size() - 1,
        static_cast<size_t>((values.size() - 1) * q + 0.5));
    return values[idx];
}

double mean(const std::vector<double>& values)
{
    if (values.empty()) return 0.0;
    return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

ml::JointArray get_q(const mjData* data, const std::vector<int>& qpos_idx)
{
    ml::JointArray q{};
    for (int i = 0; i < ml::kNumJoints; ++i) q[i] = static_cast<float>(data->qpos[7 + qpos_idx[i]]);
    return q;
}

ml::JointArray get_dq(const mjData* data, const std::vector<int>& qvel_idx)
{
    ml::JointArray dq{};
    for (int i = 0; i < ml::kNumJoints; ++i) dq[i] = static_cast<float>(data->qvel[6 + qvel_idx[i]]);
    return dq;
}

void set_sim_state(
    mjData* data,
    const std::vector<int>& qpos_idx,
    const std::vector<int>& qvel_idx,
    const ml::RobotSnapshot& snap)
{
    data->qpos[3] = snap.quat[0];
    data->qpos[4] = snap.quat[1];
    data->qpos[5] = snap.quat[2];
    data->qpos[6] = snap.quat[3];
    data->qvel[3] = snap.ang_vel[0];
    data->qvel[4] = snap.ang_vel[1];
    data->qvel[5] = snap.ang_vel[2];
    for (int i = 0; i < ml::kNumJoints; ++i) {
        data->qpos[7 + qpos_idx[i]] = snap.q[i];
        data->qvel[6 + qvel_idx[i]] = snap.dq[i];
    }
}

double apply_pd(
    const Args& args,
    const mjModel* model,
    mjData* data,
    const std::vector<int>& qpos_idx,
    const std::vector<int>& qvel_idx,
    const ml::JointArray& target,
    const ml::JointArray& kp,
    const ml::JointArray& kd,
    const ml::JointArray& tau_limit)
{
    double max_abs_tau = 0.0;
    for (int i = 0; i < model->nu && i < ml::kNumJoints; ++i) {
        const double q = data->qpos[7 + qpos_idx[i]];
        const double dq = data->qvel[6 + qvel_idx[i]];
        const double target_q = (i == kHeadMotorIndex) ? 0.0 : static_cast<double>(target[i]);
        double tau = (target_q - q) * kp[i] - dq * kd[i];
        if (!std::isfinite(tau)) tau = -args.damping_kd * dq;
        double lo = -std::max(1.0f, tau_limit[i]);
        double hi = std::max(1.0f, tau_limit[i]);
        if (model->actuator_ctrllimited && model->actuator_ctrllimited[i]) {
            lo = std::max(lo, model->actuator_ctrlrange[2 * i + 0]);
            hi = std::min(hi, model->actuator_ctrlrange[2 * i + 1]);
        }
        tau = std::clamp(tau, lo, hi);
        data->ctrl[i] = tau;
        max_abs_tau = std::max(max_abs_tau, std::fabs(tau));
    }
    return max_abs_tau;
}

double max_abs_ctrl(const mjModel* model, const mjData* data)
{
    double out = 0.0;
    for (int i = 0; i < model->nu; ++i) {
        out = std::max(out, std::fabs(data->ctrl[i]));
    }
    return out;
}

struct PushStepResult {
    bool force_active{false};
    bool impulse_applied{false};
};

PushStepResult apply_push_disturbance(const Args& args, SimContext& sim, bool& impulse_done)
{
    PushStepResult result;
    if (sim.model->nbody <= 0 || sim.data->xfrc_applied == nullptr) return result;

    mju_zero(sim.data->xfrc_applied, 6 * sim.model->nbody);
    if (sim.push_body_id < 0) return result;

    const double sim_time = sim.data->time;
    const double dt = std::max(1e-9, sim.model->opt.timestep);
    double* xfrc = sim.data->xfrc_applied + 6 * sim.push_body_id;

    const bool force_active =
        has_vec3(args.push_force) &&
        sim_time + 0.5 * dt >= args.push_start_s &&
        sim_time < args.push_start_s + args.push_duration_s;
    if (force_active) {
        for (int i = 0; i < 3; ++i) xfrc[i] += args.push_force[static_cast<size_t>(i)];
        result.force_active = true;
    }

    const bool should_apply_impulse =
        has_vec3(args.push_impulse) &&
        !impulse_done &&
        sim_time + 0.5 * dt >= args.push_impulse_time_s;
    if (should_apply_impulse) {
        for (int i = 0; i < 3; ++i) xfrc[i] += args.push_impulse[static_cast<size_t>(i)] / dt;
        impulse_done = true;
        result.impulse_applied = true;
    }

    return result;
}

double max_abs(const ml::JointArray& values)
{
    double out = 0.0;
    for (float v : values) out = std::max(out, static_cast<double>(std::fabs(v)));
    return out;
}

std::string json_escape(const std::string& value)
{
    std::ostringstream out;
    for (char c : value) {
        switch (c) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << c; break;
        }
    }
    return out.str();
}

std::string summary_json(const Summary& s)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(6)
        << "{"
        << "\"mode\":\"" << s.mode << "\","
        << "\"real_forward_only\":" << (s.real_forward_only ? "true" : "false") << ","
        << "\"duration_s\":" << s.duration_s << ","
        << "\"elapsed_s\":" << s.elapsed_s << ","
        << "\"sim_steps\":" << s.sim_steps << ","
        << "\"control_steps\":" << s.control_steps << ","
        << "\"sim_hz\":" << s.sim_hz << ","
        << "\"control_hz\":" << s.control_hz << ","
        << "\"target_sim_hz\":" << s.target_sim_hz << ","
        << "\"target_control_hz\":" << s.target_control_hz << ","
        << "\"simulation_dt\":" << s.simulation_dt << ","
        << "\"control_decimation\":" << s.control_decimation << ","
        << "\"deadline_misses\":" << s.deadline_misses << ","
        << "\"infer_mean_ms\":" << s.infer_mean_ms << ","
        << "\"infer_p95_ms\":" << s.infer_p95_ms << ","
        << "\"infer_p99_ms\":" << s.infer_p99_ms << ","
        << "\"infer_max_ms\":" << s.infer_max_ms << ","
        << "\"state_age_mean_ms\":" << s.state_age_mean_ms << ","
        << "\"state_age_max_ms\":" << s.state_age_max_ms << ","
        << "\"max_abs_tau\":" << s.max_abs_tau << ","
        << "\"max_abs_q\":" << s.max_abs_q << ","
        << "\"max_abs_dq\":" << s.max_abs_dq << ","
        << "\"min_base_height\":" << s.min_base_height << ","
        << "\"max_gravity_xy\":" << s.max_gravity_xy << ","
        << "\"max_root_xy_drift\":" << s.max_root_xy_drift << ","
        << "\"max_policy_target_jump\":" << s.max_policy_target_jump << ","
        << "\"push_body\":\"" << json_escape(s.push_body) << "\","
        << "\"push_enabled\":" << (s.push_enabled ? "true" : "false") << ","
        << "\"push_start_s\":" << s.push_start_s << ","
        << "\"push_duration_s\":" << s.push_duration_s << ","
        << "\"push_impulse_time_s\":" << s.push_impulse_time_s << ","
        << "\"push_force_norm\":" << s.push_force_norm << ","
        << "\"push_impulse_norm\":" << s.push_impulse_norm << ","
        << "\"push_force_steps\":" << s.push_force_steps << ","
        << "\"push_impulse_applied\":" << (s.push_impulse_applied ? "true" : "false") << ","
        << "\"pass\":" << (s.pass ? "true" : "false") << ","
        << "\"fail_reason\":\"" << json_escape(s.fail_reason) << "\""
        << "}";
    return out.str();
}

void evaluate_closed_loop(const Args& args, Summary& s)
{
    std::vector<std::string> failures;
    auto add_failure = [&](const std::string& label, double value, const char* op, double threshold) {
        std::ostringstream oss;
        oss << label << "=" << value << " " << op << " " << threshold;
        failures.push_back(oss.str());
    };

    if (args.min_control_hz > 0.0 && s.control_hz < args.min_control_hz) {
        add_failure("control_hz", s.control_hz, "<", args.min_control_hz);
    }
    if (args.max_deadline_miss_ratio >= 0.0) {
        const double denom = std::max(1, s.sim_steps);
        const double ratio = static_cast<double>(s.deadline_misses) / denom;
        if (ratio > args.max_deadline_miss_ratio) {
            add_failure("deadline_miss_ratio", ratio, ">", args.max_deadline_miss_ratio);
        }
    }
    if (args.max_infer_p99_ms > 0.0 && s.infer_p99_ms > args.max_infer_p99_ms) {
        add_failure("infer_p99_ms", s.infer_p99_ms, ">", args.max_infer_p99_ms);
    }
    if (args.min_base_height > 0.0 && s.min_base_height < args.min_base_height) {
        add_failure("min_base_height", s.min_base_height, "<", args.min_base_height);
    }
    if (args.max_gravity_xy > 0.0 && s.max_gravity_xy > args.max_gravity_xy) {
        add_failure("max_gravity_xy", s.max_gravity_xy, ">", args.max_gravity_xy);
    }
    if (args.max_root_xy_drift > 0.0 && s.max_root_xy_drift > args.max_root_xy_drift) {
        add_failure("max_root_xy_drift", s.max_root_xy_drift, ">", args.max_root_xy_drift);
    }
    if (args.max_abs_dq_limit > 0.0 && s.max_abs_dq > args.max_abs_dq_limit) {
        add_failure("max_abs_dq", s.max_abs_dq, ">", args.max_abs_dq_limit);
    }
    if (args.max_abs_tau_limit > 0.0 && s.max_abs_tau > args.max_abs_tau_limit) {
        add_failure("max_abs_tau", s.max_abs_tau, ">", args.max_abs_tau_limit);
    }

    s.pass = failures.empty();
    if (!failures.empty()) {
        std::ostringstream reason;
        for (size_t i = 0; i < failures.size(); ++i) {
            if (i > 0) reason << "; ";
            reason << failures[i];
        }
        s.fail_reason = reason.str();
    }
}

SimContext make_sim(const Args& args, const fs::path& root, double sim_dt, double base_height)
{
    fs::path xml_path = resolve_path(root, args.xml.empty() ? fs::path("assets/robots/magicbot_z1/scene.xml")
                                                            : fs::path(args.xml));
    char error[1024] = {0};
    mjModel* model = mj_loadXML(xml_path.string().c_str(), nullptr, error, sizeof(error));
    if (!model) throw std::runtime_error(std::string("mj_loadXML failed: ") + error);
    mjData* data = mj_makeData(model);
    if (!data) {
        mj_deleteModel(model);
        throw std::runtime_error("mj_makeData failed");
    }
    model->opt.timestep = sim_dt;

    SimContext sim;
    sim.model = model;
    sim.data = data;
    sim.qpos_idx = actuator_qpos_indices(model);
    sim.qvel_idx = actuator_qvel_indices(model);
    sim.root_body_id = mj_name2id(model, mjOBJ_BODY, "pelvis");
    sim.push_body_id = mj_name2id(model, mjOBJ_BODY, args.push_body.c_str());
    if ((has_vec3(args.push_force) || has_vec3(args.push_impulse)) && sim.push_body_id < 0) {
        throw std::runtime_error("push body not found in MuJoCo model: " + args.push_body);
    }
    if (args.ground_correction) {
        sim.floor_geom_id = mj_name2id(model, mjOBJ_GEOM, args.ground_floor_geom.c_str());
        sim.ground_contact_geom_ids = resolve_contact_geom_ids(model, args.ground_body_keywords);
        if (sim.floor_geom_id < 0) {
            std::cerr << "[GroundCorrection][WARN] floor geom not found: "
                      << args.ground_floor_geom << std::endl;
        }
    }
    if (model->nu != ml::kNumJoints) {
        throw std::runtime_error("expected 24 actuators, got " + std::to_string(model->nu));
    }

    const fs::path pose_path = resolve_path(root, args.initial_pose_yaml);
    const auto mj2lab = load_mj2lab(pose_path, model->nu);
    sim.have_init_q = load_initial_pose(pose_path, mj2lab, sim.init_q);
    if (base_height > 0.0) data->qpos[2] = base_height;
    if (sim.have_init_q) {
        for (int i = 0; i < ml::kNumJoints; ++i) {
            data->qpos[7 + sim.qpos_idx[i]] = sim.init_q[i];
        }
    }
    mj_forward(model, data);
    return sim;
}

void destroy_sim(SimContext& sim)
{
    if (sim.data) mj_deleteData(sim.data);
    if (sim.model) mj_deleteModel(sim.model);
    sim.data = nullptr;
    sim.model = nullptr;
}

Summary run_rate_loop(
    Args& args,
    const ml::LocoConfig& cfg,
    ml::ControllerCore& core,
    SimContext& sim)
{
    const bool real = args.mode == "real-state-sim";
    const double control_dt = sim.model->opt.timestep * args.control_decimation;
    int target_steps = 1;
    if (real && args.real_forward_only) {
        target_steps = std::max(1, static_cast<int>(std::round(args.duration / control_dt)));
    } else {
        target_steps = std::max(1, static_cast<int>(std::round(args.duration / sim.model->opt.timestep)));
    }

    ml::JointArray policy_target = sim.have_init_q ? sim.init_q : cfg.default_motor();
    core.seed_target(policy_target);
    core.reset_policy();
    const ml::Command command{{args.vx, args.vy, args.wz}};
    ml::MujocoSimAdapterOptions sim_adapter_options;
    sim_adapter_options.qpos_idx = sim.qpos_idx;
    sim_adapter_options.qvel_idx = sim.qvel_idx;
    ml::MujocoSimAdapter sim_adapter(sim.model, sim.data, sim_adapter_options);
    ml::ControllerRuntime sim_runtime(core, sim_adapter);
    std::vector<double> infer_ms;
    std::vector<double> state_age_ms;
    int missed_deadline = 0;
    int executed_steps = 0;
    int control_steps = 0;
    double max_abs_tau = 0.0;
    double max_abs_q = 0.0;
    double max_abs_dq = 0.0;
    double min_base_height = std::numeric_limits<double>::infinity();
    double max_gravity_xy = 0.0;
    double max_root_xy_drift = 0.0;
    double max_policy_target_jump = 0.0;
    int push_force_steps = 0;
    bool push_impulse_done = false;
    const double root_x0 = sim.data->qpos[0];
    const double root_y0 = sim.data->qpos[1];
    ml::JointArray previous_policy_target = policy_target;
    bool have_previous_policy_target = false;
    bool requested_loco = false;

#ifdef ENABLE_MAGICBOT_SDK
    ml::MagicbotSdkAdapter robot;
    ml::SdkRobotState robot_state;
    bool real_connected = false;
    if (real) {
        if (!args.skip_network_check && !ml::local_ip_exists(args.local_ip)) {
            throw std::runtime_error("local IP " + args.local_ip + " is not assigned to this machine");
        }
        robot.initialize_and_connect(args.local_ip);
        robot.enter_lowlevel(robot_state);
        robot_state.wait_ready(args.state_timeout);
        real_connected = true;
        std::cout << "[real-state-sim] state ready; no joint commands will be published" << std::endl;
    }
#else
    if (real) {
        throw std::runtime_error("real-state-sim was not built because MagicBot SDK was not found");
    }
#endif

    const double start = now_sec();
    double next_t = start;
    try {
        for (int step = 0; step < target_steps && g_running.load(); ++step) {
            ++executed_steps;
            const bool control_tick =
                !real || (real && args.real_forward_only) || (step % args.control_decimation == 0);

            if (control_tick) {
                ml::RuntimeTickOutput tick;

#ifdef ENABLE_MAGICBOT_SDK
                if (real) {
                    ml::RobotSnapshot snap = robot_state.snapshot();
                    set_sim_state(sim.data, sim.qpos_idx, sim.qvel_idx, snap);
                    mj_forward(sim.model, sim.data);
                    const double age = robot_state.state_age_ms();
                    if (age >= 0.0) state_age_ms.push_back(age);
                    const auto t0 = std::chrono::steady_clock::now();
                    const auto mode_request = requested_loco
                                                  ? ml::ModeRequest::none()
                                                  : ml::mode_request_for_control_mode(ml::ControlMode::Loco);
                    tick.snapshot = snap;
                    tick.core = core.step(snap, command, mode_request, static_cast<float>(control_dt));
                    tick.adapter.backend = "real-state-sim";
                    const auto t1 = std::chrono::steady_clock::now();
                    if (tick.core.telemetry.policy_evaluated) {
                        infer_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
                    }
                } else
#endif
                {
                    ml::RuntimeTickInput tick_input;
                    tick_input.command = command;
                    tick_input.mode_request = requested_loco
                                                  ? ml::ModeRequest::none()
                                                  : ml::mode_request_for_control_mode(ml::ControlMode::Loco);
                    tick_input.control_dt_s = static_cast<float>(sim.model->opt.timestep);
                    tick_input.publish_target = true;
                    const auto t0 = std::chrono::steady_clock::now();
                    tick = sim_runtime.tick(tick_input);
                    const auto t1 = std::chrono::steady_clock::now();
                    if (tick.core.telemetry.policy_evaluated) {
                        infer_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
                    }
                }

                const auto gravity = tick.core.telemetry.projected_gravity;
                max_gravity_xy = std::max(
                    max_gravity_xy,
                    std::sqrt(static_cast<double>(gravity[0]) * gravity[0] +
                              static_cast<double>(gravity[1]) * gravity[1]));
                requested_loco = true;
                if (tick.core.telemetry.policy_evaluated && have_previous_policy_target) {
                    double jump = 0.0;
                    for (int i = 0; i < ml::kNumJoints; ++i) {
                        jump = std::max(
                            jump,
                            static_cast<double>(
                                std::fabs(tick.core.telemetry.raw_policy_target[i] - previous_policy_target[i])));
                    }
                    max_policy_target_jump = std::max(max_policy_target_jump, jump);
                }
                if (tick.core.telemetry.policy_evaluated) {
                    previous_policy_target = tick.core.telemetry.raw_policy_target;
                    have_previous_policy_target = true;
                    ++control_steps;
                }
                policy_target = tick.core.target.q;
                max_abs_q = std::max(max_abs_q, max_abs(tick.snapshot.q));
                max_abs_dq = std::max(max_abs_dq, max_abs(tick.snapshot.dq));
            }

            if (!(real && args.real_forward_only)) {
                if (real) {
                    max_abs_tau = std::max(
                        max_abs_tau,
                        apply_pd(
                            args,
                            sim.model,
                            sim.data,
                            sim.qpos_idx,
                            sim.qvel_idx,
                            policy_target,
                            core.gains().kp,
                            core.gains().kd,
                            core.gains().tau_limit));
                } else {
                    max_abs_tau = std::max(max_abs_tau, max_abs_ctrl(sim.model, sim.data));
                }
                const PushStepResult push = apply_push_disturbance(args, sim, push_impulse_done);
                if (push.force_active) ++push_force_steps;
                mj_step(sim.model, sim.data);
                if (args.ground_correction) {
                    (void)correct_ground_penetration(
                        sim.model,
                        sim.data,
                        sim.floor_geom_id,
                        sim.ground_contact_geom_ids,
                        args.ground_max_penetration);
                }
                max_abs_q = std::max(max_abs_q, max_abs(get_q(sim.data, sim.qpos_idx)));
                max_abs_dq = std::max(max_abs_dq, max_abs(get_dq(sim.data, sim.qvel_idx)));
            }
            min_base_height = std::min(min_base_height, sim.data->qpos[2]);
            max_root_xy_drift = std::max(
                max_root_xy_drift,
                std::hypot(sim.data->qpos[0] - root_x0, sim.data->qpos[1] - root_y0));

            if (args.realtime) {
                const double period = (real && args.real_forward_only) ? control_dt : sim.model->opt.timestep;
                next_t += period;
                const double remain = next_t - now_sec();
                if (remain > 0.0) {
                    sleep_sec(remain);
                } else {
                    ++missed_deadline;
                    next_t = now_sec();
                }
            }
        }
    } catch (...) {
#ifdef ENABLE_MAGICBOT_SDK
        if (real_connected) robot.disconnect(!args.real_disconnect);
#endif
        throw;
    }

#ifdef ENABLE_MAGICBOT_SDK
    if (real_connected) robot.disconnect(!args.real_disconnect);
#endif

    const double elapsed = now_sec() - start;
    Summary s;
    s.mode = args.mode;
    s.real_forward_only = args.real_forward_only;
    s.duration_s = args.duration;
    s.elapsed_s = elapsed;
    s.sim_steps = executed_steps;
    s.control_steps = control_steps;
    s.sim_hz = executed_steps / std::max(elapsed, 1e-9);
    s.control_hz = control_steps / std::max(elapsed, 1e-9);
    s.target_sim_hz = (real && args.real_forward_only) ? 1.0 / control_dt : 1.0 / sim.model->opt.timestep;
    s.target_control_hz = 1.0 / control_dt;
    s.simulation_dt = sim.model->opt.timestep;
    s.control_decimation = args.control_decimation;
    s.deadline_misses = missed_deadline;
    s.infer_mean_ms = mean(infer_ms);
    s.infer_p95_ms = percentile(infer_ms, 0.95);
    s.infer_p99_ms = percentile(infer_ms, 0.99);
    s.infer_max_ms = infer_ms.empty() ? 0.0 : *std::max_element(infer_ms.begin(), infer_ms.end());
    if (!state_age_ms.empty()) {
        s.state_age_mean_ms = mean(state_age_ms);
        s.state_age_max_ms = *std::max_element(state_age_ms.begin(), state_age_ms.end());
    }
    s.max_abs_tau = max_abs_tau;
    s.max_abs_q = max_abs_q;
    s.max_abs_dq = max_abs_dq;
    s.min_base_height = std::isfinite(min_base_height) ? min_base_height : 0.0;
    s.max_gravity_xy = max_gravity_xy;
    s.max_root_xy_drift = max_root_xy_drift;
    s.max_policy_target_jump = max_policy_target_jump;
    s.push_body = args.push_body;
    s.push_enabled = has_vec3(args.push_force) || has_vec3(args.push_impulse);
    s.push_start_s = args.push_start_s;
    s.push_duration_s = args.push_duration_s;
    s.push_impulse_time_s = args.push_impulse_time_s;
    s.push_force_norm = vec3_norm(args.push_force);
    s.push_impulse_norm = vec3_norm(args.push_impulse);
    s.push_force_steps = push_force_steps;
    s.push_impulse_applied = push_impulse_done;
    if (args.closed_loop_check) {
        evaluate_closed_loop(args, s);
    }
    return s;
}

}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        Args args = parse_args(argc, argv);
        fs::path root = repo_root();
        double yaml_sim_dt = 0.002;
        int yaml_decimation = 10;
        double base_height = 0.69;
        load_mujoco_yaml(args, root, yaml_sim_dt, yaml_decimation, base_height);
        if (args.sim_dt <= 0.0) args.sim_dt = yaml_sim_dt;
        if (args.control_decimation <= 0) args.control_decimation = yaml_decimation;
        args.control_decimation = std::max(1, args.control_decimation);
        if (args.closed_loop_check && args.min_control_hz <= 0.0) {
            args.min_control_hz = 0.85 / (args.sim_dt * static_cast<double>(args.control_decimation));
        }

        const fs::path config_path = resolve_path(root, args.config);
        ml::LocoConfig cfg = ml::load_loco_config(config_path);
        ml::ControllerCoreOptions core_options;
        core_options.safety.enabled = false;
        ml::ControllerCore core(cfg, core_options);
        core.warmup(3);

        SimContext sim = make_sim(args, root, args.sim_dt, base_height);
        std::cout << "=== MagicBot Z1 Native Dual Inference Rate ===\n"
                  << "mode=" << args.mode
                  << " sim_dt=" << sim.model->opt.timestep
                  << " control_decimation=" << args.control_decimation
                  << " config=" << config_path << "\n";

        Summary summary = run_rate_loop(args, cfg, core, sim);
        const std::string json = summary_json(summary);
        std::cout << "RATE_SUMMARY " << json << std::endl;
        if (!args.summary_json.empty()) {
            std::ofstream out(resolve_path(root, args.summary_json));
            out << json << "\n";
        }
        destroy_sim(sim);
        if (args.closed_loop_check && !summary.pass) {
            std::cerr << "[ClosedLoopCheck] FAILED: " << summary.fail_reason << std::endl;
            return 2;
        }
        if (args.closed_loop_check) {
            std::cout << "[ClosedLoopCheck] PASSED" << std::endl;
        }
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "[Error] " << exc.what() << std::endl;
        return 1;
    }
}
