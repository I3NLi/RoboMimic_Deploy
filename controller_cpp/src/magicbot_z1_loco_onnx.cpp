#include "controller_core.h"
#include "controller_runtime.h"
#include "magicbot_loco_core.h"
#include "magicbot_real_adapter.h"
#include "magicbot_loco_sdk_adapter.h"
#include "native_external_policy_registry.h"
#include "text_control_command.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <filesystem>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <linux/joystick.h>
#include <memory>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include <arpa/inet.h>

namespace ml = magicbot_loco;

namespace {

std::atomic<bool> g_running{true};

void signal_handler(int signum)
{
    std::cout << "[Signal] Received " << signum << ", stopping" << std::endl;
    g_running.store(false);
}

struct Args {
    std::filesystem::path config{"policies/loco_mode/config/LocoMode_lowKp.yaml"};
    std::filesystem::path beyond_yaml{};
    std::filesystem::path track_mimic_yaml{};
    bool dry_run{false};
    bool connect_check{false};
    bool read_state{false};
    bool run{false};
    bool input_check{false};
    bool debug_entry{false};
    bool debug_entry_only{false};
    std::string debug_entry_tts{
        "Entering real robot debug mode. Switching to passive test mode in three seconds. Please keep clear."};
    double debug_entry_wait_s{3.0};
    double debug_entry_passive_s{2.0};
    bool tts_required{false};
    bool try_gait_passive{false};
    bool skip_gait_passive{false};
    bool skip_lowlevel_disconnect{false};
    bool hard_exit_after_final_damping{false};

    std::string local_ip;
    bool skip_network_check{false};
    float vx{0.0f};
    float vy{0.0f};
    float wz{0.0f};
    bool allow_loco{false};
    bool allow_dance{false};
    bool allow_skill{false};
    bool keyboard_control{false};
    bool gamepad_control{false};
    bool udp_control{false};
    std::string gamepad_device{"/dev/input/js0"};
    std::string udp_bind{"0.0.0.0"};
    int udp_port{15000};
    double udp_timeout_s{0.35};
    float input_step{0.05f};
    float input_deadzone{0.08f};
    int gamepad_axis_vx{1};
    int gamepad_axis_vy{0};
    int gamepad_axis_wz{3};
    float gamepad_axis_vx_sign{-1.0f};
    float gamepad_axis_vy_sign{-1.0f};
    float gamepad_axis_wz_sign{-1.0f};
    int gamepad_deadman_button{4};
    int gamepad_stop_button{1};
    int gamepad_loco_button{0};
    int gamepad_stand_button{3};
    int gamepad_zero_button{2};
    int gamepad_pause_button{7};
    int gamepad_reset_button{6};
    int gamepad_dance_button{-1};
    int gamepad_skill_button{-1};
    double state_timeout{10.0};
    std::string prepare_gait{"recovery_stand"};
    double stand_time{2.0};
    double pre_stand_hold_s{1.0};
    double final_stand_time{1.0};
    double final_stand_hold_s{0.5};
    bool hold_final_stand{false};
    bool pd_stand_only{false};
    double duration{3.0};
    float stand_kp_scale{0.5f};
    float kp_scale{1.0f};
    float kd_scale{1.0f};
    float max_target_rate{25.0f};
    float joint_limit_margin{0.01f};
    float damping_kd{3.0f};
    ml::RateWatchdogConfig rate;
    ml::SafetyConfig safety;
    double log_interval{1.0};
};

void print_usage(const char* argv0)
{
    std::cout
        << "Usage: " << argv0 << " [--dry-run|--connect-check|--read-state|--run|--debug-entry-only] [options]\n"
        << "\n"
        << "Native MagicBot Z1 loco runner. ONNX policy and 500Hz command publish run in C++.\n"
        << "\n"
        << "Common:\n"
        << "  --config PATH                    Loco YAML config\n"
        << "  --beyond-yaml PATH               Enable BeyondMimic as DANCE external policy\n"
        << "  --track-mimic-yaml PATH          Enable BeyondMimic trajectory/TrackMimic as SKILL policy\n"
        << "  --local-ip IP                    SDK local IP, default MAGICBOT_LOCAL_IP or 192.168.54.119\n"
        << "  --skip-network-check             Do not verify local-ip exists on this host\n"
        << "  --dry-run                        Load YAML/ONNX and run one inference\n"
        << "  --connect-check                  Connect/disconnect only, no LowLevel switch\n"
        << "  --read-state                     LowLevel state subscription only, no command publish\n"
        << "  --run                            Publish low-level commands\n"
        << "  --input-check                    Read keyboard/gamepad input only, no robot connection\n"
        << "  --pd-stand-only                  With --run, hold default PD stand and never run ONNX\n"
        << "  --allow-loco                     Required before ONNX loco is allowed\n"
        << "  --allow-dance                    Required before DANCE/BeyondMimic is allowed on real robot\n"
        << "  --allow-skill                    Required before BeyondMimic trajectory/TrackMimic is allowed on real robot\n"
        << "\n"
        << "Motion:\n"
        << "  --vx V --vy V --wz V             Normalized command inputs; YAML cmd_range maps physical speed\n"
        << "  --duration S                     Run duration; <=0 means until stopped\n"
        << "  --stand-time S                   Interpolate to default stand before mode\n"
        << "  --pre-stand-hold-s S             Hold stand before loco starts\n"
        << "  --final-stand-time S             Return-to-stand time on normal exit\n"
        << "  --final-stand-hold-s S           Stand hold before final damping\n"
        << "  --hold-final-stand               Hold final stand until signal\n"
        << "  --max-target-rate R              Max target slew rate in rad/s, default 25\n"
        << "\n"
        << "Operator input:\n"
        << "  --keyboard-control               Live terminal keyboard input in run loop\n"
        << "                                   L stand/loco, R re-stand, W/S vx, Q/E vy, A/D wz,\n"
        << "                                   B beyond/dance, T track/skill, X zero, Space/P pause-zero, Esc stop\n"
        << "  --gamepad-control                Live Linux joystick input in run loop\n"
        << "  --gamepad-device PATH            Joystick device, default /dev/input/js0\n"
        << "  --udp-control                    Live UDP command input in run loop\n"
        << "  --udp-bind IP                    UDP bind address, default 0.0.0.0\n"
        << "  --udp-port N                     UDP command port, default 15000\n"
        << "  --udp-timeout-s S                Zero command after no UDP packet, default 0.35\n"
        << "  --input-step V                   Keyboard normalized command step, default 0.05\n"
        << "  --input-deadzone V               Gamepad axis deadzone, default 0.08\n"
        << "  --gamepad-axis-vx N              Axis index for vx, default 1\n"
        << "  --gamepad-axis-vy N              Axis index for vy, default 0\n"
        << "  --gamepad-axis-wz N              Axis index for wz, default 3\n"
        << "  --gamepad-axis-vx-sign S         Axis sign for vx, default -1\n"
        << "  --gamepad-axis-vy-sign S         Axis sign for vy, default -1\n"
        << "  --gamepad-axis-wz-sign S         Axis sign for wz, default -1\n"
        << "  --gamepad-deadman-button N       Button that must be held for nonzero command, default 4\n"
        << "  --gamepad-stop-button N          Button that stops run loop, default 1\n"
        << "  --gamepad-loco-button N          Button that enters LOCO, default 0\n"
        << "  --gamepad-stand-button N         Button that enters STAND, default 3\n"
        << "  --gamepad-zero-button N          Button that pause-zeros command, default 2\n"
        << "  --gamepad-pause-button N         Button that toggles pause-zero, default 7\n"
        << "  --gamepad-reset-button N         Button that re-interpolates to STAND, default 6\n"
        << "  --gamepad-dance-button N         Optional button that enters DANCE/BeyondMimic, default disabled\n"
        << "  --gamepad-skill-button N         Optional button that enters BeyondMimic trajectory/TrackMimic, default disabled\n"
        << "\n"
        << "Debug entry:\n"
        << "  --debug-entry                    TTS, wait, then LowLevel passive damping before run\n"
        << "  --debug-entry-only               Only run debug-entry passive damping sequence\n"
        << "  --debug-entry-tts TEXT           TTS prompt\n"
        << "  --debug-entry-wait-s S           Wait after TTS\n"
        << "  --debug-entry-passive-s S        LowLevel damping duration\n";
}

std::string take_value(int& i, int argc, char** argv)
{
    if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + argv[i]);
    return argv[++i];
}

Args parse_args(int argc, char** argv)
{
    Args args;
    const char* env_ip = std::getenv("MAGICBOT_LOCAL_IP");
    args.local_ip = env_ip && *env_ip ? env_ip : "192.168.54.119";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else if (a == "--config") {
            args.config = take_value(i, argc, argv);
        } else if (a == "--beyond-yaml") {
            args.beyond_yaml = take_value(i, argc, argv);
        } else if (a == "--track-mimic-yaml") {
            args.track_mimic_yaml = take_value(i, argc, argv);
        } else if (a == "--dry-run") {
            args.dry_run = true;
        } else if (a == "--connect-check") {
            args.connect_check = true;
        } else if (a == "--read-state") {
            args.read_state = true;
        } else if (a == "--run") {
            args.run = true;
        } else if (a == "--input-check") {
            args.input_check = true;
        } else if (a == "--debug-entry") {
            args.debug_entry = true;
        } else if (a == "--debug-entry-only") {
            args.debug_entry_only = true;
        } else if (a == "--debug-entry-tts") {
            args.debug_entry_tts = take_value(i, argc, argv);
        } else if (a == "--debug-entry-wait-s") {
            args.debug_entry_wait_s = std::stod(take_value(i, argc, argv));
        } else if (a == "--debug-entry-passive-s") {
            args.debug_entry_passive_s = std::stod(take_value(i, argc, argv));
        } else if (a == "--tts-required") {
            args.tts_required = true;
        } else if (a == "--try-gait-passive") {
            args.try_gait_passive = true;
        } else if (a == "--skip-gait-passive") {
            args.skip_gait_passive = true;
        } else if (a == "--skip-lowlevel-disconnect") {
            args.skip_lowlevel_disconnect = true;
        } else if (a == "--hard-exit-after-final-damping") {
            args.hard_exit_after_final_damping = true;
        } else if (a == "--local-ip") {
            args.local_ip = take_value(i, argc, argv);
        } else if (a == "--skip-network-check") {
            args.skip_network_check = true;
        } else if (a == "--vx") {
            args.vx = std::stof(take_value(i, argc, argv));
        } else if (a == "--vy") {
            args.vy = std::stof(take_value(i, argc, argv));
        } else if (a == "--wz") {
            args.wz = std::stof(take_value(i, argc, argv));
        } else if (a == "--allow-loco") {
            args.allow_loco = true;
        } else if (a == "--allow-dance") {
            args.allow_dance = true;
        } else if (a == "--allow-skill") {
            args.allow_skill = true;
        } else if (a == "--keyboard-control") {
            args.keyboard_control = true;
        } else if (a == "--gamepad-control") {
            args.gamepad_control = true;
        } else if (a == "--udp-control") {
            args.udp_control = true;
        } else if (a == "--gamepad-device") {
            args.gamepad_device = take_value(i, argc, argv);
        } else if (a == "--udp-bind") {
            args.udp_bind = take_value(i, argc, argv);
        } else if (a == "--udp-port") {
            args.udp_port = std::stoi(take_value(i, argc, argv));
        } else if (a == "--udp-timeout-s") {
            args.udp_timeout_s = std::stod(take_value(i, argc, argv));
        } else if (a == "--input-step") {
            args.input_step = std::stof(take_value(i, argc, argv));
        } else if (a == "--input-deadzone") {
            args.input_deadzone = std::stof(take_value(i, argc, argv));
        } else if (a == "--gamepad-axis-vx") {
            args.gamepad_axis_vx = std::stoi(take_value(i, argc, argv));
        } else if (a == "--gamepad-axis-vy") {
            args.gamepad_axis_vy = std::stoi(take_value(i, argc, argv));
        } else if (a == "--gamepad-axis-wz") {
            args.gamepad_axis_wz = std::stoi(take_value(i, argc, argv));
        } else if (a == "--gamepad-axis-vx-sign") {
            args.gamepad_axis_vx_sign = std::stof(take_value(i, argc, argv));
        } else if (a == "--gamepad-axis-vy-sign") {
            args.gamepad_axis_vy_sign = std::stof(take_value(i, argc, argv));
        } else if (a == "--gamepad-axis-wz-sign") {
            args.gamepad_axis_wz_sign = std::stof(take_value(i, argc, argv));
        } else if (a == "--gamepad-deadman-button") {
            args.gamepad_deadman_button = std::stoi(take_value(i, argc, argv));
        } else if (a == "--gamepad-stop-button") {
            args.gamepad_stop_button = std::stoi(take_value(i, argc, argv));
        } else if (a == "--gamepad-loco-button") {
            args.gamepad_loco_button = std::stoi(take_value(i, argc, argv));
        } else if (a == "--gamepad-stand-button") {
            args.gamepad_stand_button = std::stoi(take_value(i, argc, argv));
        } else if (a == "--gamepad-zero-button") {
            args.gamepad_zero_button = std::stoi(take_value(i, argc, argv));
        } else if (a == "--gamepad-pause-button") {
            args.gamepad_pause_button = std::stoi(take_value(i, argc, argv));
        } else if (a == "--gamepad-reset-button") {
            args.gamepad_reset_button = std::stoi(take_value(i, argc, argv));
        } else if (a == "--gamepad-dance-button") {
            args.gamepad_dance_button = std::stoi(take_value(i, argc, argv));
        } else if (a == "--gamepad-skill-button") {
            args.gamepad_skill_button = std::stoi(take_value(i, argc, argv));
        } else if (a == "--state-timeout") {
            args.state_timeout = std::stod(take_value(i, argc, argv));
        } else if (a == "--prepare-gait") {
            args.prepare_gait = take_value(i, argc, argv);
        } else if (a == "--stand-time") {
            args.stand_time = std::stod(take_value(i, argc, argv));
        } else if (a == "--pre-stand-hold-s") {
            args.pre_stand_hold_s = std::stod(take_value(i, argc, argv));
        } else if (a == "--final-stand-time") {
            args.final_stand_time = std::stod(take_value(i, argc, argv));
        } else if (a == "--final-stand-hold-s") {
            args.final_stand_hold_s = std::stod(take_value(i, argc, argv));
        } else if (a == "--hold-final-stand") {
            args.hold_final_stand = true;
        } else if (a == "--stand-only" || a == "--pd-stand-only") {
            args.pd_stand_only = true;
        } else if (a == "--duration") {
            args.duration = std::stod(take_value(i, argc, argv));
        } else if (a == "--stand-kp-scale") {
            args.stand_kp_scale = std::stof(take_value(i, argc, argv));
        } else if (a == "--kp-scale") {
            args.kp_scale = std::stof(take_value(i, argc, argv));
        } else if (a == "--kd-scale") {
            args.kd_scale = std::stof(take_value(i, argc, argv));
        } else if (a == "--max-target-rate") {
            args.max_target_rate = std::stof(take_value(i, argc, argv));
        } else if (a == "--joint-limit-margin") {
            args.joint_limit_margin = std::stof(take_value(i, argc, argv));
        } else if (a == "--damping-kd") {
            args.damping_kd = std::stof(take_value(i, argc, argv));
        } else if (a == "--disable-rate-watchdog") {
            args.rate.enabled = false;
        } else if (a == "--rate-watchdog-min-hz") {
            args.rate.min_hz = std::stod(take_value(i, argc, argv));
        } else if (a == "--rate-watchdog-window") {
            args.rate.window_s = std::stod(take_value(i, argc, argv));
        } else if (a == "--rate-watchdog-max-gap-ms") {
            args.rate.max_gap_s = std::stod(take_value(i, argc, argv)) / 1000.0;
        } else if (a == "--disable-motion-safety") {
            args.safety.enabled = false;
        } else if (a == "--motion-safety-joint-scope") {
            args.safety.joint_scope = take_value(i, argc, argv);
        } else if (a == "--motion-max-joint-vel") {
            args.safety.max_joint_vel = std::stof(take_value(i, argc, argv));
        } else if (a == "--motion-max-ang-vel") {
            args.safety.max_ang_vel = std::stof(take_value(i, argc, argv));
        } else if (a == "--motion-max-gravity-xy") {
            args.safety.max_gravity_xy = std::stof(take_value(i, argc, argv));
        } else if (a == "--motion-max-default-dev") {
            args.safety.max_default_dev = std::stof(take_value(i, argc, argv));
        } else if (a == "--motion-max-target-error") {
            args.safety.max_target_error = std::stof(take_value(i, argc, argv));
        } else if (a == "--motion-max-policy-target-dev") {
            args.safety.max_policy_target_dev = std::stof(take_value(i, argc, argv));
        } else if (a == "--motion-max-policy-target-jump") {
            args.safety.max_policy_target_jump = std::stof(take_value(i, argc, argv));
        } else if (a == "--log-interval") {
            args.log_interval = std::stod(take_value(i, argc, argv));
        } else {
            throw std::runtime_error("unknown argument: " + a);
        }
    }

    const int selected = static_cast<int>(args.dry_run) + static_cast<int>(args.connect_check) +
                         static_cast<int>(args.read_state) + static_cast<int>(args.run) +
                         static_cast<int>(args.input_check) +
                         static_cast<int>(args.debug_entry_only);
    if (selected == 0) args.dry_run = true;
    if (selected > 1) {
        throw std::runtime_error(
            "use only one of --dry-run, --connect-check, --read-state, --run, --input-check, --debug-entry-only");
    }
    if (args.run && !args.pd_stand_only && !args.allow_loco) {
        throw std::runtime_error("refusing ONNX loco without --allow-loco; use --pd-stand-only for PD standing");
    }
    if (args.safety.joint_scope != "body" && args.safety.joint_scope != "legs" && args.safety.joint_scope != "all") {
        throw std::runtime_error("--motion-safety-joint-scope must be body, legs or all");
    }
    const int live_inputs = static_cast<int>(args.keyboard_control) + static_cast<int>(args.gamepad_control) +
                            static_cast<int>(args.udp_control);
    if (live_inputs > 1) {
        throw std::runtime_error("use only one of --keyboard-control, --gamepad-control or --udp-control");
    }
    args.input_step = std::clamp(args.input_step, 0.001f, 1.0f);
    args.input_deadzone = std::clamp(args.input_deadzone, 0.0f, 0.95f);
    args.udp_port = std::clamp(args.udp_port, 1, 65535);
    args.udp_timeout_s = std::clamp(args.udp_timeout_s, 0.02, 10.0);
    return args;
}

void check_network_preflight(const Args& args)
{
    if (args.skip_network_check) {
        std::cerr << "[WARN] Skipping local IP preflight check" << std::endl;
        return;
    }
    if (!ml::local_ip_exists(args.local_ip)) {
        throw std::runtime_error("local IP " + args.local_ip + " is not assigned to this machine");
    }
}

std::string range_string(const ml::JointArray& a)
{
    auto [min_it, max_it] = std::minmax_element(a.begin(), a.end());
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << "[" << *min_it << ".." << *max_it << "]";
    return oss.str();
}

float clamp_command(float v)
{
    return std::clamp(v, -1.0f, 1.0f);
}

float normalized_axis_value(int value)
{
    const float denom = value < 0 ? 32768.0f : 32767.0f;
    return clamp_command(static_cast<float>(value) / denom);
}

float apply_axis_deadzone(float value, float deadzone)
{
    const float dz = std::clamp(deadzone, 0.0f, 0.95f);
    const float av = std::fabs(value);
    if (av <= dz) return 0.0f;
    return std::copysign((av - dz) / std::max(1e-6f, 1.0f - dz), value);
}

struct LiveInputState {
    std::array<float, 3> command{0.0f, 0.0f, 0.0f};
    ml::ModeRequest mode_request{ml::ModeRequest::none()};
    bool stop_requested{false};
    bool toggle_loco_requested{false};
    bool reset_stand_requested{false};
    bool pause_zero{false};
    bool changed{false};
    bool zeroed_by_deadman{false};
    std::string status;
};

void set_live_input_mode_request(LiveInputState& out, ml::ModeRequest request)
{
    if (!request.requested) return;
    out.mode_request = request;
}

bool live_input_requested_mode(const LiveInputState& input, ml::ControlMode mode)
{
    return input.mode_request.requested && input.mode_request.mode == mode;
}

bool dance_request_allowed(const Args& args, const char* prefix)
{
    if (!args.allow_dance) {
        std::cout << prefix
                  << " DANCE ignored; add --allow-dance together with --beyond-yaml PATH to enable BeyondMimic"
                  << std::endl;
        return false;
    }
    if (args.beyond_yaml.empty()) {
        std::cout << prefix << " DANCE ignored; start with --beyond-yaml PATH to enable BeyondMimic"
                  << std::endl;
        return false;
    }
    return true;
}

bool skill_request_allowed(const Args& args, const char* prefix)
{
    if (!args.allow_skill) {
        std::cout << prefix
                  << " SKILL ignored; add --allow-skill together with --track-mimic-yaml PATH to enable BeyondMimic trajectory/TrackMimic"
                  << std::endl;
        return false;
    }
    if (args.track_mimic_yaml.empty()) {
        std::cout << prefix << " SKILL ignored; start with --track-mimic-yaml PATH to enable BeyondMimic trajectory/TrackMimic"
                  << std::endl;
        return false;
    }
    return true;
}

class TerminalKeyboardInput {
public:
    TerminalKeyboardInput(std::array<float, 3> initial_command, float step)
        : command_(initial_command), step_(step)
    {
        if (!isatty(STDIN_FILENO)) {
            throw std::runtime_error("--keyboard-control requires a TTY stdin");
        }
        if (tcgetattr(STDIN_FILENO, &old_term_) != 0) {
            throw std::runtime_error(std::string("tcgetattr failed: ") + std::strerror(errno));
        }
        old_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (old_flags_ < 0) {
            throw std::runtime_error(std::string("fcntl(F_GETFL) failed: ") + std::strerror(errno));
        }

        termios raw = old_term_;
        raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
            throw std::runtime_error(std::string("tcsetattr failed: ") + std::strerror(errno));
        }
        if (fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK) != 0) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
            throw std::runtime_error(std::string("fcntl(F_SETFL) failed: ") + std::strerror(errno));
        }
        active_ = true;
    }

    ~TerminalKeyboardInput()
    {
        if (!active_) return;
        tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
        fcntl(STDIN_FILENO, F_SETFL, old_flags_);
    }

    LiveInputState poll()
    {
        LiveInputState out;
        char ch = 0;
        while (true) {
            const ssize_t n = read(STDIN_FILENO, &ch, 1);
            if (n == 1) {
                handle_char(ch, out);
                continue;
            }
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                throw std::runtime_error(std::string("keyboard read failed: ") + std::strerror(errno));
            }
            break;
        }
        out.command = paused_ ? std::array<float, 3>{0.0f, 0.0f, 0.0f} : command_;
        out.pause_zero = paused_;
        out.status = paused_ ? "keyboard paused" : "keyboard";
        return out;
    }

private:
    void handle_char(char ch, LiveInputState& out)
    {
        switch (ch) {
        case 3:
        case 27:
            out.stop_requested = true;
            out.changed = true;
            return;
        case 'l':
        case 'L':
            out.toggle_loco_requested = true;
            break;
        case 'b':
        case 'B':
            set_live_input_mode_request(out, ml::mode_request_for_control_mode(ml::ControlMode::Dance));
            paused_ = false;
            break;
        case 't':
        case 'T':
            set_live_input_mode_request(out, ml::mode_request_for_control_mode(ml::ControlMode::Skill));
            paused_ = false;
            break;
        case 'm':
        case 'M':
            set_live_input_mode_request(out, ml::mode_request_for_control_mode(ml::ControlMode::Passive));
            paused_ = false;
            break;
        case 'f':
        case 'F':
            set_live_input_mode_request(out, ml::mode_request_for_control_mode(ml::ControlMode::FinalDamping));
            paused_ = false;
            break;
        case 'r':
        case 'R':
            out.reset_stand_requested = true;
            break;
        case 'w':
        case 'W':
            command_[0] = clamp_command(command_[0] + step_);
            break;
        case 's':
        case 'S':
            command_[0] = clamp_command(command_[0] - step_);
            break;
        case 'q':
        case 'Q':
            command_[1] = clamp_command(command_[1] + step_);
            break;
        case 'e':
        case 'E':
            command_[1] = clamp_command(command_[1] - step_);
            break;
        case 'a':
        case 'A':
            command_[2] = clamp_command(command_[2] + step_);
            break;
        case 'd':
        case 'D':
            command_[2] = clamp_command(command_[2] - step_);
            break;
        case 'x':
        case 'X':
        case '0':
            command_ = {0.0f, 0.0f, 0.0f};
            break;
        case ' ':
        case 'p':
        case 'P':
            paused_ = !paused_;
            break;
        default:
            return;
        }
        out.changed = true;
    }

    std::array<float, 3> command_{0.0f, 0.0f, 0.0f};
    float step_{0.05f};
    termios old_term_{};
    int old_flags_{0};
    bool active_{false};
    bool paused_{false};
};

class GamepadInput {
public:
    explicit GamepadInput(const Args& args)
        : args_(args), axes_(16, 0.0f), buttons_(16, 0)
    {
        fd_ = open(args_.gamepad_device.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            throw std::runtime_error("failed to open " + args_.gamepad_device + ": " + std::strerror(errno));
        }
    }

    ~GamepadInput()
    {
        if (fd_ >= 0) close(fd_);
    }

    LiveInputState poll()
    {
        LiveInputState out;
        js_event event{};
        while (true) {
            const ssize_t n = read(fd_, &event, sizeof(event));
            if (n == static_cast<ssize_t>(sizeof(event))) {
                handle_event(event, out);
                continue;
            }
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                throw std::runtime_error(std::string("gamepad read failed: ") + std::strerror(errno));
            }
            break;
        }

        const bool deadman_required = args_.gamepad_deadman_button >= 0;
        const bool deadman_pressed = !deadman_required || button_pressed(args_.gamepad_deadman_button);
        out.command = {
            axis_command(args_.gamepad_axis_vx, args_.gamepad_axis_vx_sign),
            axis_command(args_.gamepad_axis_vy, args_.gamepad_axis_vy_sign),
            axis_command(args_.gamepad_axis_wz, args_.gamepad_axis_wz_sign),
        };
        if (!deadman_pressed) {
            out.command = {0.0f, 0.0f, 0.0f};
            out.zeroed_by_deadman = true;
        }
        if (paused_) {
            out.command = {0.0f, 0.0f, 0.0f};
            out.pause_zero = true;
        }
        out.status = paused_ ? "gamepad paused" : (deadman_pressed ? "gamepad" : "gamepad deadman-open");
        return out;
    }

private:
    void handle_event(const js_event& event, LiveInputState& out)
    {
        const uint8_t type = event.type & static_cast<uint8_t>(~JS_EVENT_INIT);
        if (type == JS_EVENT_AXIS) {
            if (event.number >= axes_.size()) axes_.resize(static_cast<size_t>(event.number) + 1, 0.0f);
            axes_[event.number] = normalized_axis_value(event.value);
            out.changed = true;
        } else if (type == JS_EVENT_BUTTON) {
            if (event.number >= buttons_.size()) buttons_.resize(static_cast<size_t>(event.number) + 1, 0);
            buttons_[event.number] = event.value ? 1 : 0;
            out.changed = true;
            if (args_.gamepad_stop_button >= 0 && event.number == args_.gamepad_stop_button && event.value) {
                out.stop_requested = true;
            }
            if (event.value) {
                if (args_.gamepad_loco_button >= 0 && event.number == args_.gamepad_loco_button) {
                    paused_ = false;
                    set_live_input_mode_request(out, ml::mode_request_for_control_mode(ml::ControlMode::Loco));
                }
                if (args_.gamepad_stand_button >= 0 && event.number == args_.gamepad_stand_button) {
                    paused_ = false;
                    set_live_input_mode_request(out, ml::mode_request_for_control_mode(ml::ControlMode::Stand));
                }
                if (args_.gamepad_zero_button >= 0 && event.number == args_.gamepad_zero_button) {
                    paused_ = true;
                }
                if (args_.gamepad_pause_button >= 0 && event.number == args_.gamepad_pause_button) {
                    paused_ = !paused_;
                }
                if (args_.gamepad_reset_button >= 0 && event.number == args_.gamepad_reset_button) {
                    paused_ = false;
                    out.reset_stand_requested = true;
                }
                if (args_.gamepad_dance_button >= 0 && event.number == args_.gamepad_dance_button) {
                    paused_ = false;
                    set_live_input_mode_request(out, ml::mode_request_for_control_mode(ml::ControlMode::Dance));
                }
                if (args_.gamepad_skill_button >= 0 && event.number == args_.gamepad_skill_button) {
                    paused_ = false;
                    set_live_input_mode_request(out, ml::mode_request_for_control_mode(ml::ControlMode::Skill));
                }
            }
        }
    }

    bool button_pressed(int index) const
    {
        return index >= 0 && index < static_cast<int>(buttons_.size()) && buttons_[static_cast<size_t>(index)] != 0;
    }

    float axis_command(int index, float sign) const
    {
        if (index < 0 || index >= static_cast<int>(axes_.size())) return 0.0f;
        return clamp_command(apply_axis_deadzone(axes_[static_cast<size_t>(index)], args_.input_deadzone) * sign);
    }

    const Args& args_;
    int fd_{-1};
    std::vector<float> axes_;
    std::vector<int> buttons_;
    bool paused_{false};
};

class UdpCommandInput {
public:
    UdpCommandInput(const Args& args, std::array<float, 3> initial_command)
        : args_(args), command_(initial_command)
    {
        fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) throw std::runtime_error(std::string("udp socket failed: ") + std::strerror(errno));

        const int flags = fcntl(fd_, F_GETFL, 0);
        if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) != 0) {
            throw std::runtime_error(std::string("udp fcntl failed: ") + std::strerror(errno));
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(args_.udp_port));
        if (args_.udp_bind.empty() || args_.udp_bind == "0.0.0.0") {
            addr.sin_addr.s_addr = htonl(INADDR_ANY);
        } else if (inet_pton(AF_INET, args_.udp_bind.c_str(), &addr.sin_addr) != 1) {
            throw std::runtime_error("invalid --udp-bind address: " + args_.udp_bind);
        }

        if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            throw std::runtime_error(
                "udp bind " + args_.udp_bind + ":" + std::to_string(args_.udp_port) +
                " failed: " + std::strerror(errno));
        }
        last_packet_t_ = std::chrono::steady_clock::now();
    }

    ~UdpCommandInput()
    {
        if (fd_ >= 0) close(fd_);
    }

    LiveInputState poll()
    {
        LiveInputState out;
        char buffer[256];
        while (true) {
            sockaddr_in src{};
            socklen_t src_len = sizeof(src);
            const ssize_t n = recvfrom(fd_, buffer, sizeof(buffer) - 1, 0, reinterpret_cast<sockaddr*>(&src), &src_len);
            if (n > 0) {
                buffer[n] = '\0';
                handle_message(std::string(buffer, static_cast<size_t>(n)), out);
                continue;
            }
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                throw std::runtime_error(std::string("udp recvfrom failed: ") + std::strerror(errno));
            }
            break;
        }

        const auto now = std::chrono::steady_clock::now();
        const bool timed_out = have_packet_ &&
                               std::chrono::duration<double>(now - last_packet_t_).count() > args_.udp_timeout_s;
        if (timed_out && command_ != std::array<float, 3>{0.0f, 0.0f, 0.0f}) {
            command_ = {0.0f, 0.0f, 0.0f};
            out.changed = true;
            out.status = "udp timeout";
        }

        out.command = paused_ || timed_out ? std::array<float, 3>{0.0f, 0.0f, 0.0f} : command_;
        out.pause_zero = paused_;
        if (out.status.empty()) out.status = timed_out ? "udp timeout" : "udp";
        return out;
    }

private:
    void handle_message(std::string message, LiveInputState& out)
    {
        std::array<float, 3> cmd = command_;
        for (const ml::TextControlOperation& op : ml::parse_text_control_operations(std::move(message))) {
            if (op.type == ml::TextControlOperation::Type::Velocity) {
                if (op.axis >= 0 && op.axis < 3) {
                    cmd[static_cast<size_t>(op.axis)] = op.value;
                }
            } else {
                handle_action(op.action, out, cmd);
            }
        }

        command_ = cmd;
        have_packet_ = true;
        last_packet_t_ = std::chrono::steady_clock::now();
        out.changed = true;
        out.status = "udp packet";
    }

    void handle_action(ml::TextControlAction action, LiveInputState& out, std::array<float, 3>& cmd)
    {
        const ml::TextControlActionEffect effect = ml::text_control_action_effect(action);
        if (effect.zero_command) {
            cmd = {0.0f, 0.0f, 0.0f};
        }
        if (effect.pause) {
            paused_ = true;
        }
        if (effect.unpause) {
            paused_ = false;
        }
        if (effect.stop) {
            out.stop_requested = true;
        }
        if (effect.toggle_loco) {
            out.toggle_loco_requested = true;
        }
        if (effect.reset_stand) {
            out.reset_stand_requested = true;
        }
        if (!effect.mode_requested) {
            return;
        }
        set_live_input_mode_request(out, ml::mode_request_for_text_control_effect(effect));
    }

    const Args& args_;
    std::array<float, 3> command_{0.0f, 0.0f, 0.0f};
    int fd_{-1};
    bool paused_{false};
    bool have_packet_{false};
    std::chrono::steady_clock::time_point last_packet_t_{};
};

class OperatorInput {
public:
    OperatorInput(const Args& args, std::array<float, 3> initial_command)
    {
        if (args.keyboard_control) {
            keyboard_ = std::make_unique<TerminalKeyboardInput>(initial_command, args.input_step);
            std::cout << "[Input] Keyboard control enabled: L stand/loco, M passive, F final damping, B dance, T skill, "
                         "R re-stand, W/S vx, Q/E vy, A/D wz, X zero, Space/P pause-zero, Esc stop"
                      << std::endl;
        }
        if (args.gamepad_control) {
            gamepad_ = std::make_unique<GamepadInput>(args);
            std::cout << "[Input] Gamepad control enabled: device=" << args.gamepad_device
                      << " axes(vx,vy,wz)=[" << args.gamepad_axis_vx << " " << args.gamepad_axis_vy << " "
                      << args.gamepad_axis_wz << "] deadman_button=" << args.gamepad_deadman_button
                      << " stop_button=" << args.gamepad_stop_button
                      << " loco_button=" << args.gamepad_loco_button
                      << " stand_button=" << args.gamepad_stand_button
                      << " zero_button=" << args.gamepad_zero_button
                      << " pause_button=" << args.gamepad_pause_button
                      << " reset_button=" << args.gamepad_reset_button
                      << " dance_button=" << args.gamepad_dance_button
                      << " skill_button=" << args.gamepad_skill_button << std::endl;
        }
        if (args.udp_control) {
            udp_ = std::make_unique<UdpCommandInput>(args, initial_command);
            std::cout << "[Input] UDP command enabled: bind=" << args.udp_bind << ":" << args.udp_port
                      << " timeout_s=" << args.udp_timeout_s
                      << " format='vx vy wz [loco|stand|passive|final_damping|beyond|stop]'"
                         " or 'vx=... vy=... wz=... mode=loco'"
                      << std::endl;
        }
    }

    bool enabled() const { return keyboard_ != nullptr || gamepad_ != nullptr || udp_ != nullptr; }

    LiveInputState poll()
    {
        if (keyboard_) return keyboard_->poll();
        if (gamepad_) return gamepad_->poll();
        if (udp_) return udp_->poll();
        return {};
    }

private:
    std::unique_ptr<TerminalKeyboardInput> keyboard_;
    std::unique_ptr<GamepadInput> gamepad_;
    std::unique_ptr<UdpCommandInput> udp_;
};

void dry_run(const ml::LocoConfig& cfg, ml::OnnxLocoPolicy& policy)
{
    ml::JointArray q = cfg.default_motor();
    ml::JointArray dq{};
    std::array<float, 3> zero3{0.0f, 0.0f, 0.0f};
    std::array<float, 3> gravity{0.0f, 0.0f, -1.0f};
    auto result = policy.infer(q, dq, zero3, gravity, zero3);
    auto [min_a, max_a] = std::minmax_element(result.action_lab.begin(), result.action_lab.end());
    std::cout << "Config: " << cfg.config_path << "\n"
              << "ONNX: " << cfg.policy_path << "\n"
              << "ONNX input/output: " << cfg.num_obs << " -> " << cfg.num_actions << "\n"
              << "Action sample range: " << *min_a << " .. " << *max_a << "\n"
              << "Target sample range: " << range_string(result.target_motor) << std::endl;
}

ml::RobotSnapshot dry_run_snapshot(const ml::LocoConfig& cfg)
{
    ml::RobotSnapshot snapshot;
    snapshot.q = cfg.default_motor();
    snapshot.quat = {1.0f, 0.0f, 0.0f, 0.0f};
    snapshot.counts = {12, 8, 1, 1};
    return snapshot;
}

void dry_run_external_policy(
    ml::ControllerCore& core,
    const ml::LocoConfig& cfg,
    ml::ControlMode mode,
    const std::string& policy_key,
    const std::filesystem::path& yaml_path,
    const std::string& label)
{
    const auto output = core.step(
        dry_run_snapshot(cfg),
        ml::Command{},
        ml::mode_request_for_control_mode(mode, policy_key),
        static_cast<float>(cfg.policy_dt));
    if (output.telemetry.mode != mode) {
        throw std::runtime_error(label + " dry-run selected unexpected mode");
    }
    if (output.telemetry.external_policy != policy_key) {
        throw std::runtime_error(label + " dry-run selected unexpected external policy");
    }
    if (!output.telemetry.policy_evaluated) {
        throw std::runtime_error(label + " dry-run did not evaluate external policy");
    }
    std::cout << "[DryRun] " << label << " loaded through shared registry: "
              << yaml_path << std::endl;
}

ml::JointArray stand_interpolation(
    ml::MagicbotSdkAdapter& robot,
    ml::SdkRobotState& state,
    const ml::LocoConfig& cfg,
    const Args& args,
    ml::RateWatchdog& rate_watchdog)
{
    ml::RobotSnapshot snap = state.snapshot();
    ml::JointArray command_target = snap.q;
    const ml::JointArray target_default = cfg.default_motor();
    const ml::JointArray kp_motor_base = cfg.kps_motor();
    const ml::JointArray kd_motor = cfg.kds_motor();
    const ml::JointArray tau_limit = cfg.tau_limit_motor();
    ml::JointArray kp_motor{};
    for (int i = 0; i < ml::kNumJoints; ++i) kp_motor[i] = kp_motor_base[i] * args.stand_kp_scale;

    const int steps = std::max(1, static_cast<int>(args.stand_time * ml::kControlHz));
    std::cout << "[Stage] Standing interpolation: " << args.stand_time << "s, " << steps << " steps" << std::endl;
    auto next_t = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(ml::kControlDt));
    for (int step = 0; step < steps && g_running.load(); ++step) {
        rate_watchdog.check();
        snap = state.snapshot();
        const float alpha = static_cast<float>(step + 1) / static_cast<float>(steps);
        ml::JointArray raw_target{};
        for (int i = 0; i < ml::kNumJoints; ++i) {
            raw_target[i] = (1.0f - alpha) * snap.q[i] + alpha * target_default[i];
        }
        const auto limited = ml::torque_limited_target(
            raw_target, snap.q, snap.dq, kp_motor, kd_motor, tau_limit, cfg.tau_limit_scale);
        command_target = ml::clamp_and_rate_limit(
            limited, command_target, args.max_target_rate, static_cast<float>(ml::kControlDt), 0.01f);
        robot.publish_sdk24_command(snap.counts, command_target, kp_motor, kd_motor, false, args.damping_kd);
        next_t += period;
        std::this_thread::sleep_until(next_t);
    }
    return command_target;
}

ml::JointArray hold_default_stand(
    ml::MagicbotSdkAdapter& robot,
    ml::SdkRobotState& state,
    const ml::LocoConfig& cfg,
    const Args& args,
    ml::RateWatchdog& rate_watchdog,
    ml::MotionSafety& safety,
    ml::JointArray command_target,
    double duration_s,
    const std::string& label,
    bool hold_until_stopped)
{
    const ml::JointArray target_default = cfg.default_motor();
    const ml::JointArray kp_base = cfg.kps_motor();
    const ml::JointArray kd_base = cfg.kds_motor();
    const ml::JointArray tau_limit = cfg.tau_limit_motor();
    ml::JointArray kp_motor{};
    ml::JointArray kd_motor{};
    for (int i = 0; i < ml::kNumJoints; ++i) {
        kp_motor[i] = kp_base[i] * args.kp_scale;
        kd_motor[i] = kd_base[i] * args.kd_scale;
    }

    std::cout << "[Stage] Holding default stand (" << label << "): "
              << (hold_until_stopped ? "until stopped" : std::to_string(duration_s) + "s") << std::endl;
    auto start = std::chrono::steady_clock::now();
    auto next_t = start;
    auto last_log = start - std::chrono::seconds(60);
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(ml::kControlDt));
    while (g_running.load()) {
        const auto now = std::chrono::steady_clock::now();
        if (!hold_until_stopped && std::chrono::duration<double>(now - start).count() >= duration_s) break;
        rate_watchdog.check();
        const auto snap = state.snapshot();
        const auto limited =
            ml::torque_limited_target(target_default, snap.q, snap.dq, kp_motor, kd_motor, tau_limit, cfg.tau_limit_scale);
        command_target = ml::clamp_and_rate_limit(
            limited,
            command_target,
            args.max_target_rate,
            static_cast<float>(ml::kControlDt),
            args.joint_limit_margin);
        safety.check(snap, &command_target, &target_default, nullptr);
        robot.publish_sdk24_command(snap.counts, command_target, kp_motor, kd_motor, false, args.damping_kd);
        if (std::chrono::duration<double>(now - last_log).count() >= args.log_interval) {
            std::cout << "[" << label << "] q=" << range_string(snap.q) << " target=" << range_string(command_target)
                      << " counts=" << ml::counts_string(snap.counts) << std::endl;
            last_log = now;
        }
        next_t += period;
        std::this_thread::sleep_until(next_t);
    }
    return command_target;
}

void publish_damping_for_duration(
    ml::MagicbotSdkAdapter& robot,
    ml::SdkRobotState& state,
    const Args& args,
    ml::RateWatchdog& rate_watchdog,
    double duration_s)
{
    const int steps = std::max(1, static_cast<int>(duration_s * ml::kControlHz));
    std::cout << "[Stage] Holding LowLevel passive damping for " << duration_s << "s (" << steps << " steps)"
              << std::endl;
    auto next_t = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(ml::kControlDt));
    for (int i = 0; i < steps && g_running.load(); ++i) {
        rate_watchdog.check();
        robot.publish_damping(state.snapshot().counts, args.damping_kd);
        next_t += period;
        std::this_thread::sleep_until(next_t);
    }
}

int connect_check(const Args& args)
{
    check_network_preflight(args);
    ml::MagicbotSdkAdapter robot;
    robot.initialize_and_connect(args.local_ip);
    std::cout << "[ConnectCheck] Passed" << std::endl;
    robot.disconnect(false);
    return 0;
}

int read_state_only(const Args& args)
{
    check_network_preflight(args);
    ml::SdkRobotState state;
    ml::MagicbotSdkAdapter robot;
    robot.initialize_and_connect(args.local_ip);
    robot.prepare_gait(args.prepare_gait);
    robot.enter_lowlevel(state);
    state.wait_ready(args.state_timeout);
    std::cout << "[ReadState] Robot state ready. No JointCommand will be published." << std::endl;

    auto start = std::chrono::steady_clock::now();
    auto last_log = start - std::chrono::seconds(60);
    while (g_running.load()) {
        const auto now = std::chrono::steady_clock::now();
        if (args.duration > 0.0 && std::chrono::duration<double>(now - start).count() >= args.duration) break;
        if (std::chrono::duration<double>(now - last_log).count() >= args.log_interval) {
            const auto snap = state.snapshot();
            std::cout << "[ReadState] counts=" << ml::counts_string(snap.counts) << " q=" << range_string(snap.q)
                      << " dq=" << range_string(snap.dq) << " quat=[" << snap.quat[0] << " " << snap.quat[1]
                      << " " << snap.quat[2] << " " << snap.quat[3] << "]" << std::endl;
            last_log = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    robot.disconnect(args.skip_lowlevel_disconnect);
    return 0;
}

int input_check_only(const Args& args)
{
    if (!args.keyboard_control && !args.gamepad_control && !args.udp_control) {
        throw std::runtime_error("--input-check requires --keyboard-control, --gamepad-control or --udp-control");
    }
    std::array<float, 3> initial_cmd{args.vx, args.vy, args.wz};
    OperatorInput input(args, initial_cmd);
    std::cout << "[InputCheck] No robot connection. Press Esc/stop button or wait for duration." << std::endl;

    ml::ControlMode mode = ml::ControlMode::Stand;
    const auto start = std::chrono::steady_clock::now();
    auto last_log = start - std::chrono::seconds(60);
    while (g_running.load()) {
        const auto now = std::chrono::steady_clock::now();
        if (args.duration > 0.0 && std::chrono::duration<double>(now - start).count() >= args.duration) break;
        const auto state = input.poll();
        if (state.stop_requested) {
            std::cout << "[InputCheck] Stop requested" << std::endl;
            break;
        }
        if (state.toggle_loco_requested) {
            mode = mode == ml::ControlMode::Loco ? ml::ControlMode::Stand : ml::ControlMode::Loco;
        }
        if (state.reset_stand_requested) {
            mode = ml::ControlMode::Stand;
        }
        if (state.mode_request.requested) {
            bool allowed = true;
            if (state.mode_request.mode == ml::ControlMode::Dance) {
                allowed = dance_request_allowed(args, "[InputCheck]");
            } else if (state.mode_request.mode == ml::ControlMode::Skill) {
                allowed = skill_request_allowed(args, "[InputCheck]");
            }
            if (allowed) {
                mode = state.mode_request.mode;
            }
        }
        if (state.changed || std::chrono::duration<double>(now - last_log).count() >= args.log_interval) {
            std::cout << "[InputCheck] " << state.status << " mode=" << ml::control_mode_name(mode)
                      << " cmd=[" << state.command[0] << " " << state.command[1] << " " << state.command[2] << "]";
            if (state.zeroed_by_deadman) std::cout << " deadman=open";
            if (state.pause_zero) std::cout << " pause-zero";
            if (state.reset_stand_requested) std::cout << " reset-stand";
            if (live_input_requested_mode(state, ml::ControlMode::Passive)) std::cout << " passive";
            if (live_input_requested_mode(state, ml::ControlMode::Dance)) std::cout << " dance";
            if (live_input_requested_mode(state, ml::ControlMode::Skill)) std::cout << " skill";
            if (live_input_requested_mode(state, ml::ControlMode::FinalDamping)) std::cout << " final-damping";
            std::cout << std::endl;
            last_log = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}

int run_robot_with_finally(const Args& args, const ml::LocoConfig& cfg, ml::ControllerCore& core)
{
    check_network_preflight(args);
    ml::SdkRobotState state;
    ml::MagicbotSdkAdapter robot;
    bool entered_lowlevel = false;
    int rc = 0;
    int hard_exit_code = -1;

    auto final_damping = [&]() {
        if (!entered_lowlevel) return;
        try {
            std::cout << "[Final] Publishing final damping command" << std::endl;
            ml::MagicbotRealAdapter real_adapter(robot, state);
            ml::ControllerRuntime runtime(core, real_adapter);
            runtime.write_damping(args.damping_kd);
            std::cout << "[Final] Final damping command published" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } catch (const std::exception& exc) {
            std::cerr << "[Final][WARN] Damping publish failed: " << exc.what() << std::endl;
        }
    };

    try {
        if (!args.pd_stand_only && !args.debug_entry_only) {
            const auto start = std::chrono::steady_clock::now();
            core.warmup(3);
            const auto elapsed_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            std::cout << "[Policy] Warm-up complete: 3 runs in " << elapsed_ms << "ms" << std::endl;
        }

        robot.initialize_and_connect(args.local_ip);
        const bool debug_entry = args.debug_entry || args.debug_entry_only;
        if (debug_entry) {
            robot.play_tts(args.debug_entry_tts, args.tts_required, "100000000001");
            if (args.debug_entry_wait_s > 0.0) {
                std::cout << "[Stage] Waiting " << args.debug_entry_wait_s << "s before debug/test mode" << std::endl;
                std::this_thread::sleep_for(std::chrono::duration<double>(args.debug_entry_wait_s));
            }
            if (args.try_gait_passive && !args.skip_gait_passive) {
                robot.try_highlevel_passive(false);
            } else {
                std::cout << "[Stage] Skipping HighLevel GAIT_PASSIVE; LowLevel damping fallback will be used" << std::endl;
            }
        } else {
            robot.prepare_gait(args.prepare_gait);
        }

        robot.enter_lowlevel(state);
        entered_lowlevel = true;
        state.wait_ready(args.state_timeout);
        std::cout << "[State] Robot state ready" << std::endl;

        ml::RateWatchdog rate_watchdog(args.rate);
        ml::MotionSafety safety(args.safety, cfg);

        if (args.rate.enabled) {
            std::cout << "[Safety] Rate watchdog: min_hz=" << args.rate.min_hz
                      << " window=" << args.rate.window_s
                      << " max_gap_ms=" << args.rate.max_gap_s * 1000.0 << std::endl;
        }
        if (args.safety.enabled) {
            std::cout << "[Safety] Motion safety: scope=" << args.safety.joint_scope
                      << " max_dq=" << args.safety.max_joint_vel
                      << " max_body_w=" << args.safety.max_ang_vel
                      << " max_gxy=" << args.safety.max_gravity_xy << std::endl;
        }

        if (debug_entry) {
            publish_damping_for_duration(robot, state, args, rate_watchdog, args.debug_entry_passive_s);
            if (args.debug_entry_only) {
                std::cout << "[Stage] Debug-entry passive stage complete" << std::endl;
                hard_exit_code = args.hard_exit_after_final_damping ? 0 : -1;
                rc = 0;
                final_damping();
                robot.disconnect(args.skip_lowlevel_disconnect);
                if (hard_exit_code >= 0) std::_Exit(hard_exit_code);
                return rc;
            }
        }

        ml::JointArray command_target = stand_interpolation(robot, state, cfg, args, rate_watchdog);
        command_target = hold_default_stand(
            robot,
            state,
            cfg,
            args,
            rate_watchdog,
            safety,
            command_target,
            args.pre_stand_hold_s,
            "pre-stand",
            false);

        if (args.pd_stand_only) {
            command_target = hold_default_stand(
                robot,
                state,
                cfg,
                args,
                rate_watchdog,
                safety,
                command_target,
                args.duration > 0.0 ? args.duration : 0.0,
                "pd-stand",
                args.duration <= 0.0);
        } else {
            std::array<float, 3> raw_cmd{args.vx, args.vy, args.wz};
            OperatorInput operator_input(args, raw_cmd);
            ml::ControlMode run_mode =
                operator_input.enabled() ? ml::ControlMode::Stand : ml::ControlMode::Loco;
            ml::MagicbotRealAdapter real_adapter(robot, state);
            ml::ControllerRuntime runtime(core, real_adapter);
            core.seed_target(command_target);
            core.reset_policy();
            std::string run_external_policy_key;
            ml::ModeRequest pending_mode_request = ml::mode_request_for_control_mode(run_mode);
            std::cout << "[Mode] Starting operator loop: mode=" << ml::control_mode_name(run_mode)
                      << " command=[" << args.vx << " " << args.vy << " " << args.wz << "]" << std::endl;
            if (operator_input.enabled()) {
                std::cout << "[Mode] Live input starts in STAND; request LOCO explicitly from keyboard/gamepad/udp"
                          << std::endl;
            }
            const auto loop_start = std::chrono::steady_clock::now();
            auto next_control_t = std::chrono::steady_clock::now();
            auto last_log = next_control_t - std::chrono::seconds(60);
            const auto control_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(ml::kControlDt));

            while (g_running.load()) {
                const auto now = std::chrono::steady_clock::now();
                if (args.duration > 0.0 && std::chrono::duration<double>(now - loop_start).count() >= args.duration) {
                    std::cout << "[Mode] Run duration reached" << std::endl;
                    break;
                }
                rate_watchdog.check();
                const auto snap = state.snapshot();
                safety.check(snap, &command_target, nullptr, nullptr);
                if (operator_input.enabled()) {
                    const auto input = operator_input.poll();
                    if (input.stop_requested) {
                        std::cout << "[Input] Stop requested; leaving run loop" << std::endl;
                        break;
                    }
                    raw_cmd = input.command;
                    ml::ControlMode requested_mode = run_mode;
                    bool mode_requested = false;
                    if (input.toggle_loco_requested) {
                        requested_mode =
                            run_mode == ml::ControlMode::Loco ? ml::ControlMode::Stand : ml::ControlMode::Loco;
                        mode_requested = true;
                    }
                    if (input.mode_request.requested) {
                        bool allowed = true;
                        if (input.mode_request.mode == ml::ControlMode::Dance) {
                            allowed = dance_request_allowed(args, "[Input]");
                        } else if (input.mode_request.mode == ml::ControlMode::Skill) {
                            allowed = skill_request_allowed(args, "[Input]");
                        }
                        if (allowed) {
                            requested_mode = input.mode_request.mode;
                            mode_requested = true;
                        }
                    }
                    if (input.reset_stand_requested) {
                        std::cout << "[Input] Re-stand requested" << std::endl;
                        raw_cmd = {0.0f, 0.0f, 0.0f};
                        run_mode = ml::ControlMode::Stand;
                        run_external_policy_key.clear();
                        command_target = stand_interpolation(robot, state, cfg, args, rate_watchdog);
                        core.seed_target(command_target);
                        core.reset_policy();
                        pending_mode_request = ml::mode_request_for_control_mode(ml::ControlMode::Stand);
                        next_control_t = std::chrono::steady_clock::now();
                        last_log = next_control_t - std::chrono::seconds(60);
                        continue;
                    }
                    const std::string requested_external_policy_key = input.mode_request.external_policy_key;
                    const ml::ModeRequest requested_mode_change = mode_requested
                        ? ml::mode_request_for_desired_control_mode(
                              requested_mode,
                              requested_external_policy_key,
                              run_mode,
                              run_external_policy_key)
                        : ml::ModeRequest::none();
                    if (requested_mode_change.requested) {
                        run_mode = requested_mode;
                        if (run_mode != ml::ControlMode::Loco) raw_cmd = {0.0f, 0.0f, 0.0f};
                        run_external_policy_key = requested_mode_change.external_policy_key;
                        pending_mode_request = requested_mode_change;
                        std::cout << "[Input] Mode -> " << ml::control_mode_name(run_mode) << std::endl;
                    }
                    if (input.changed && std::chrono::duration<double>(now - last_log).count() < args.log_interval) {
                        std::cout << "[Input] " << input.status << " mode=" << ml::control_mode_name(run_mode)
                                  << " cmd=[" << raw_cmd[0] << " " << raw_cmd[1] << " " << raw_cmd[2] << "]";
                        if (input.zeroed_by_deadman) std::cout << " deadman=open";
                        if (input.pause_zero) std::cout << " pause-zero";
                        if (live_input_requested_mode(input, ml::ControlMode::Passive)) std::cout << " passive";
                        if (live_input_requested_mode(input, ml::ControlMode::Dance)) std::cout << " dance";
                        if (live_input_requested_mode(input, ml::ControlMode::Skill)) std::cout << " skill";
                        if (live_input_requested_mode(input, ml::ControlMode::FinalDamping)) {
                            std::cout << " final-damping";
                        }
                        std::cout << std::endl;
                    }
                }

                ml::RuntimeTickInput tick_input;
                tick_input.command.velocity = raw_cmd;
                tick_input.mode_request = pending_mode_request;
                tick_input.control_dt_s = static_cast<float>(ml::kControlDt);
                const ml::RuntimeTickOutput tick = runtime.tick(tick_input);
                pending_mode_request = ml::ModeRequest::none();
                command_target = tick.core.target.q;
                if (std::chrono::duration<double>(now - last_log).count() >= args.log_interval) {
                    std::cout << "[" << ml::control_mode_name(tick.core.telemetry.mode) << "] policy="
                              << tick.core.telemetry.policy_steps
                              << " q=" << range_string(tick.snapshot.q)
                              << " target=" << range_string(command_target)
                              << " cmd=[" << raw_cmd[0] << " " << raw_cmd[1] << " " << raw_cmd[2] << "]"
                              << " counts=" << ml::counts_string(tick.snapshot.counts) << std::endl;
                    last_log = now;
                }
                next_control_t += control_period;
                std::this_thread::sleep_until(next_control_t);
            }
        }

        if (args.final_stand_time > 0.0 && g_running.load()) {
            Args final_args = args;
            final_args.stand_time = args.final_stand_time;
            std::cout << "[Stage] Returning to default stand before final damping" << std::endl;
            ml::JointArray command_target = state.snapshot().q;
            command_target = stand_interpolation(robot, state, cfg, final_args, rate_watchdog);
            (void)hold_default_stand(
                robot,
                state,
                cfg,
                args,
                rate_watchdog,
                safety,
                command_target,
                args.final_stand_hold_s,
                "final-stand",
                args.hold_final_stand);
        }
        rc = 0;
        hard_exit_code = args.hard_exit_after_final_damping ? 0 : -1;
    } catch (const std::exception& exc) {
        std::cerr << "[SafetyWall] " << exc.what() << "; switching to damping mode and exiting." << std::endl;
        rc = 3;
        hard_exit_code = args.hard_exit_after_final_damping ? 3 : -1;
    }

    final_damping();
    robot.disconnect(args.skip_lowlevel_disconnect);
    if (hard_exit_code >= 0) std::_Exit(hard_exit_code);
    return rc;
}

}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        Args args = parse_args(argc, argv);
        if (!std::filesystem::exists(args.config)) {
            throw std::runtime_error("config not found: " + args.config.string());
        }
        ml::LocoConfig cfg = ml::load_loco_config(args.config);

        if (args.dry_run) {
            ml::OnnxLocoPolicy policy(cfg);
            dry_run(cfg, policy);
            std::unique_ptr<ml::ControllerCore> dry_core;
            std::unique_ptr<ml::NativeBeyondMimicExternalPolicyRegistry> external_policies;
            if (!args.beyond_yaml.empty() || !args.track_mimic_yaml.empty()) {
                ml::ControllerCoreOptions dry_core_options;
                dry_core_options.safety.enabled = false;
                dry_core = std::make_unique<ml::ControllerCore>(cfg, dry_core_options);
                external_policies = std::make_unique<ml::NativeBeyondMimicExternalPolicyRegistry>(
                    static_cast<float>(cfg.policy_dt));
            }
            if (!args.beyond_yaml.empty()) {
                if (!std::filesystem::exists(args.beyond_yaml)) {
                    throw std::runtime_error("BeyondMimic config not found: " + args.beyond_yaml.string());
                }
                external_policies->register_dance(*dry_core, args.beyond_yaml.string());
                dry_run_external_policy(
                    *dry_core,
                    cfg,
                    ml::ControlMode::Dance,
                    ml::kBeyondMimicPolicyKey,
                    args.beyond_yaml,
                    "BeyondMimic");
            }
            if (!args.track_mimic_yaml.empty()) {
                if (!std::filesystem::exists(args.track_mimic_yaml)) {
                    throw std::runtime_error(
                        "BeyondMimic trajectory/TrackMimic config not found: " +
                        args.track_mimic_yaml.string());
                }
                external_policies->register_track_mimic(*dry_core, args.track_mimic_yaml.string());
                dry_run_external_policy(
                    *dry_core,
                    cfg,
                    ml::ControlMode::Skill,
                    ml::kTrackMimicPolicyKey,
                    args.track_mimic_yaml,
                    "BeyondMimic TrackMimic trajectory");
            }
            return 0;
        }
        if (args.connect_check) return connect_check(args);
        if (args.read_state) return read_state_only(args);
        if (args.input_check) return input_check_only(args);
        ml::ControllerCoreOptions core_options;
        core_options.safety = args.safety;
        core_options.kp_scale = args.kp_scale;
        core_options.kd_scale = args.kd_scale;
        core_options.max_target_rate = args.max_target_rate;
        core_options.joint_limit_margin = args.joint_limit_margin;
        core_options.damping_kd = args.damping_kd;
        ml::ControllerCore core(cfg, core_options);
        ml::NativeBeyondMimicExternalPolicyRegistry external_policies(static_cast<float>(cfg.policy_dt));
        if (!args.beyond_yaml.empty()) {
            if (!std::filesystem::exists(args.beyond_yaml)) {
                throw std::runtime_error("BeyondMimic config not found: " + args.beyond_yaml.string());
            }
            external_policies.register_dance(core, args.beyond_yaml.string());
            std::cout << "[ExternalPolicy] DANCE -> BeyondMimic: " << args.beyond_yaml << std::endl;
        }
        if (!args.track_mimic_yaml.empty()) {
            if (!std::filesystem::exists(args.track_mimic_yaml)) {
                throw std::runtime_error(
                    "BeyondMimic trajectory/TrackMimic config not found: " +
                    args.track_mimic_yaml.string());
            }
            external_policies.register_track_mimic(core, args.track_mimic_yaml.string());
            std::cout << "[ExternalPolicy] SKILL -> BeyondMimic trajectory/TrackMimic: "
                      << args.track_mimic_yaml << std::endl;
        }
        return run_robot_with_finally(args, cfg, core);
    } catch (const std::exception& exc) {
        std::cerr << "[Error] " << exc.what() << std::endl;
        return 1;
    }
}
