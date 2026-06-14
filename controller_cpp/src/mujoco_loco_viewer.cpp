#include "native_fsm_policy_types.h"
#include "beyond_mimic_policy.h"
#include "controller_core.h"
#include "controller_runtime.h"
#include "fsm_external_policy_adapter.h"
#include "magicbot_loco_core.h"
#include "mujoco_sim_adapter.h"
#include "text_control_command.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include <GL/gl.h>
#include <GL/glx.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <arpa/inet.h>
#include <mujoco/mujoco.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#ifdef None
#undef None
#endif

#ifdef ENABLE_ROS2_CAMERA
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

namespace {

struct Args {
    std::string config = "policies/loco_mode/config/LocoMode_lowKp.yaml";
    std::string beyond_yaml;
    std::string track_mimic_yaml;
    std::string mujoco_yaml = "configs/simulation/mujoco.yaml";
    std::string xml;
    std::string initial_pose_yaml;
    int width = 1280;
    int height = 900;
    int control_decimation = 10;
    int render_fps = 60;
    double sim_dt = 0.002;
    double duration = 0.0;
    double initial_base_height = 0.69;
    bool start_loco = false;
    bool paused = true;
    bool realtime = true;
    bool follow = true;
    std::string summary_json;
    bool udp_control = false;
    std::string udp_bind = "0.0.0.0";
    int udp_port = 15000;
    double udp_timeout_s = 0.35;
    std::string push_body = "pelvis";
    std::array<double, 3> push_force{0.0, 0.0, 0.0};
    std::array<double, 3> push_impulse{0.0, 0.0, 0.0};
    double push_start_s = 0.0;
    double push_duration_s = 0.0;
    double push_impulse_time_s = -1.0;
    bool camera_stream = false;
    bool camera_stream_set = false;
    std::string camera_name = "head_rgba_camera";
    std::string camera_host = "0.0.0.0";
    int camera_port = 18080;
    int camera_width = 640;
    int camera_height = 480;
    int camera_fps = 20;
    int jpeg_quality = 80;
    bool camera_ros2 = false;
    bool camera_ros2_set = false;
    bool ground_correction = false;
    std::string ground_floor_geom = "floor";
    std::vector<std::string> ground_body_keywords;
    double ground_max_penetration = 0.0;
    std::string ros2_topic_rgb = "/z1/head_camera/rgb";
    std::string ros2_topic_rgba = "/z1/head_camera/rgba";
    std::string ros2_frame_id = "head_rgba_camera";
    std::string ros2_node_name = "z1_head_camera_publisher";
    int ros2_qos_depth = 5;
    bool camera_name_set = false;
    bool camera_host_set = false;
    bool camera_port_set = false;
    bool camera_width_set = false;
    bool camera_height_set = false;
    bool camera_fps_set = false;
    bool jpeg_quality_set = false;
    bool ros2_topic_rgb_set = false;
    bool ros2_topic_rgba_set = false;
    bool ros2_frame_id_set = false;
    bool ros2_node_name_set = false;
    bool ros2_qos_depth_set = false;
};

struct XGlWindow {
    Display* display{nullptr};
    Window window{};
    Colormap colormap{};
    GLXContext context{};
    Atom wm_delete{};
    int width{1280};
    int height{900};
};

struct MouseState {
    bool left{false};
    bool middle{false};
    bool right{false};
    bool perturb{false};
    unsigned int perturb_button{0};
    int last_x{0};
    int last_y{0};
};

struct RemoteViewerDragState {
    bool perturb{false};
    bool camera{false};
    int button{0};
};

struct ViewerStats {
    int push_force_steps{0};
    bool push_impulse_applied{false};
    int mouse_perturb_steps{0};
    int http_reset_requests{0};
    int http_viewer_events{0};
    int http_control_commands{0};
    int last_perturb_body{0};
    std::string last_perturb_body_name;
    double min_base_height{std::numeric_limits<double>::infinity()};
    double max_root_xy_drift{0.0};
    double max_gravity_xy{0.0};
};

struct ViewerHttpEvent {
    std::string type;
    double x{0.0};
    double y{0.0};
    double dx{0.0};
    double dy{0.0};
    double width{1.0};
    double height{1.0};
    int button{0};
};

struct ViewerControlCommand {
    bool has_mode{false};
    magicbot_loco::TextControlAction action{magicbot_loco::TextControlAction::Zero};
    bool has_vx{false};
    bool has_vy{false};
    bool has_wz{false};
    std::array<float, 3> velocity{0.0f, 0.0f, 0.0f};
    bool has_pause{false};
    bool paused{false};
};

struct CameraStreamState {
    std::mutex mutex;
    std::vector<unsigned char> latest_jpg;
    std::vector<unsigned char> latest_png;
    std::vector<ViewerHttpEvent> viewer_events;
    std::vector<ViewerControlCommand> control_commands;
    std::string mode{"STAND"};
    std::array<float, 3> command{0.0f, 0.0f, 0.0f};
    double timestamp{0.0};
    double sim_time_s{0.0};
    double base_x{0.0};
    double base_y{0.0};
    double base_z{0.0};
    uint64_t seq{0};
    int sim_steps{0};
    int policy_steps{0};
    int push_force_steps{0};
    int mouse_perturb_steps{0};
    int http_reset_requests{0};
    int http_viewer_events{0};
    int http_control_commands{0};
    bool reset_requested{false};
    bool paused{true};
    bool push_impulse_applied{false};
    bool running{true};
};

class CameraStreamServer {
public:
    CameraStreamServer(std::string host, int port)
        : host_(std::move(host)),
          port_(port),
          state_(std::make_shared<CameraStreamState>())
    {
    }

    ~CameraStreamServer() { stop(); }

    void start()
    {
        thread_ = std::thread([this]() { serve_loop(); });
    }

    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            state_->running = false;
        }
        if (listen_fd_ >= 0) {
            shutdown(listen_fd_, SHUT_RDWR);
            close(listen_fd_);
            listen_fd_ = -1;
        }
        if (thread_.joinable()) thread_.join();
    }

    void update(std::vector<unsigned char> jpg, std::vector<unsigned char> png, double timestamp)
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->latest_jpg = std::move(jpg);
        state_->latest_png = std::move(png);
        state_->timestamp = timestamp;
        state_->seq++;
    }

    void update_status(
        std::string mode,
        bool paused,
        const std::array<float, 3>& command,
        int sim_steps,
        int policy_steps,
        double sim_time_s,
        double base_x,
        double base_y,
        double base_z,
        const ViewerStats& stats)
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->mode = std::move(mode);
        state_->paused = paused;
        state_->command = command;
        state_->sim_steps = sim_steps;
        state_->policy_steps = policy_steps;
        state_->sim_time_s = sim_time_s;
        state_->base_x = base_x;
        state_->base_y = base_y;
        state_->base_z = base_z;
        state_->push_force_steps = stats.push_force_steps;
        state_->push_impulse_applied = stats.push_impulse_applied;
        state_->mouse_perturb_steps = stats.mouse_perturb_steps;
        state_->http_reset_requests = stats.http_reset_requests;
        state_->http_viewer_events = stats.http_viewer_events;
        state_->http_control_commands = stats.http_control_commands;
    }

    bool take_reset_request()
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        const bool requested = state_->reset_requested;
        state_->reset_requested = false;
        return requested;
    }

    std::vector<ViewerHttpEvent> take_viewer_events()
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        std::vector<ViewerHttpEvent> events;
        events.swap(state_->viewer_events);
        return events;
    }

    std::vector<ViewerControlCommand> take_control_commands()
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        std::vector<ViewerControlCommand> commands;
        commands.swap(state_->control_commands);
        return commands;
    }

    std::string url() const
    {
        std::string host = host_ == "0.0.0.0" ? "127.0.0.1" : host_;
        return "http://" + host + ":" + std::to_string(port_);
    }

private:
    static void send_all(int fd, const char* data, size_t size)
    {
        size_t sent = 0;
        while (sent < size) {
            ssize_t n = send(fd, data + sent, size - sent, MSG_NOSIGNAL);
            if (n <= 0) return;
            sent += static_cast<size_t>(n);
        }
    }

    static void send_text(int fd, int code, const char* status, const std::string& body, const char* content_type)
    {
        std::string header =
            "HTTP/1.1 " + std::to_string(code) + " " + status + "\r\n"
            "Content-Type: " + content_type + "\r\n"
            "Content-Length: " + std::to_string(body.size()) + "\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Access-Control-Allow-Methods: GET,POST,OPTIONS\r\n"
            "Access-Control-Allow-Headers: content-type\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\n\r\n";
        send_all(fd, header.data(), header.size());
        send_all(fd, body.data(), body.size());
    }

    static void send_bytes(
        int fd,
        int code,
        const char* status,
        const std::vector<unsigned char>& body,
        const char* content_type)
    {
        std::string header =
            "HTTP/1.1 " + std::to_string(code) + " " + status + "\r\n"
            "Content-Type: " + content_type + "\r\n"
            "Content-Length: " + std::to_string(body.size()) + "\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Access-Control-Allow-Methods: GET,POST,OPTIONS\r\n"
            "Access-Control-Allow-Headers: content-type\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\n\r\n";
        send_all(fd, header.data(), header.size());
        if (!body.empty()) send_all(fd, reinterpret_cast<const char*>(body.data()), body.size());
    }

    static std::string request_method(const std::string& request)
    {
        const size_t first_space = request.find(' ');
        if (first_space == std::string::npos) return "";
        return request.substr(0, first_space);
    }

    static std::string request_target(const std::string& request)
    {
        const size_t first_space = request.find(' ');
        if (first_space == std::string::npos) return "/";
        const size_t second_space = request.find(' ', first_space + 1);
        if (second_space == std::string::npos) return "/";
        return request.substr(first_space + 1, second_space - first_space - 1);
    }

    static std::string request_path(const std::string& target)
    {
        const size_t query = target.find('?');
        return query == std::string::npos ? target : target.substr(0, query);
    }

    static std::string request_query(const std::string& target)
    {
        const size_t query = target.find('?');
        return query == std::string::npos ? std::string() : target.substr(query + 1);
    }

    static std::string query_value(const std::string& query, const std::string& key)
    {
        size_t start = 0;
        while (start <= query.size()) {
            const size_t end = query.find('&', start);
            const std::string token =
                query.substr(start, end == std::string::npos ? std::string::npos : end - start);
            const size_t eq = token.find('=');
            if (eq != std::string::npos && token.substr(0, eq) == key) {
                return token.substr(eq + 1);
            }
            if (end == std::string::npos) break;
            start = end + 1;
        }
        return "";
    }

    static double query_double(const std::string& query, const std::string& key, double fallback)
    {
        const std::string raw = query_value(query, key);
        if (raw.empty()) return fallback;
        char* end = nullptr;
        const double value = std::strtod(raw.c_str(), &end);
        return end != raw.c_str() && *end == '\0' && std::isfinite(value) ? value : fallback;
    }

    static int query_int(const std::string& query, const std::string& key, int fallback)
    {
        const std::string raw = query_value(query, key);
        if (raw.empty()) return fallback;
        char* end = nullptr;
        const long value = std::strtol(raw.c_str(), &end, 10);
        return end != raw.c_str() && *end == '\0' ? static_cast<int>(value) : fallback;
    }

    static std::string lower_ascii(std::string value)
    {
        return magicbot_loco::lower_ascii_copy(std::move(value));
    }

    static bool query_float_optional(
        const std::string& query,
        const std::string& key,
        float& value)
    {
        const std::string raw = query_value(query, key);
        if (raw.empty()) return false;
        char* end = nullptr;
        const float parsed = std::strtof(raw.c_str(), &end);
        if (end == raw.c_str() || *end != '\0' || !std::isfinite(parsed)) {
            throw std::runtime_error("invalid numeric control field: " + key);
        }
        value = std::clamp(parsed, -1.0f, 1.0f);
        return true;
    }

    static bool query_bool_optional(
        const std::string& query,
        const std::string& key,
        bool& value)
    {
        const std::string raw = lower_ascii(query_value(query, key));
        if (raw.empty()) return false;
        if (raw == "1" || raw == "true" || raw == "yes" || raw == "on" || raw == "pause") {
            value = true;
            return true;
        }
        if (raw == "0" || raw == "false" || raw == "no" || raw == "off" || raw == "resume") {
            value = false;
            return true;
        }
        throw std::runtime_error("invalid boolean control field: " + key);
    }

    static ViewerControlCommand parse_control_command(const std::string& query)
    {
        ViewerControlCommand command;
        bool any = false;
        const std::string mode = lower_ascii(query_value(query, "mode"));
        if (!mode.empty()) {
            magicbot_loco::TextControlAction action{};
            if (!magicbot_loco::text_control_action_from_word(mode, action)) {
                throw std::runtime_error("invalid control mode: " + mode);
            }
            command.has_mode = true;
            command.action = action;
            any = true;
        }
        if (query_float_optional(query, "vx", command.velocity[0])) {
            command.has_vx = true;
            any = true;
        }
        if (query_float_optional(query, "vy", command.velocity[1])) {
            command.has_vy = true;
            any = true;
        }
        if (query_float_optional(query, "wz", command.velocity[2])) {
            command.has_wz = true;
            any = true;
        }
        if (query_bool_optional(query, "pause", command.paused) ||
            query_bool_optional(query, "paused", command.paused)) {
            command.has_pause = true;
            any = true;
        }
        if (!any) {
            throw std::runtime_error("missing control fields");
        }
        return command;
    }

    static void handle_client(int fd, std::shared_ptr<CameraStreamState> state)
    {
        char buffer[4096] = {0};
        ssize_t n = recv(fd, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0) {
            close(fd);
            return;
        }
        const std::string request_text(buffer, static_cast<size_t>(n));
        const std::string method = request_method(request_text);
        const std::string target = request_target(request_text);
        const std::string path = request_path(target);
        const std::string query = request_query(target);

        if (method == "OPTIONS") {
            send_text(fd, 204, "No Content", "", "text/plain");
        } else if (path == "/health") {
            std::lock_guard<std::mutex> lock(state->mutex);
            std::string body = "{\"ok\":true,\"seq\":" + std::to_string(state->seq) +
                               ",\"timestamp\":" + std::to_string(state->timestamp) +
                               ",\"reset_pending\":" + (state->reset_requested ? "true" : "false") +
                               ",\"control_queue\":" + std::to_string(state->control_commands.size()) +
                               ",\"viewer_event_queue\":" + std::to_string(state->viewer_events.size()) + "}\n";
            send_text(fd, 200, "OK", body, "application/json");
        } else if (path == "/status") {
            std::lock_guard<std::mutex> lock(state->mutex);
            std::ostringstream body;
            body << std::fixed << std::setprecision(6)
                 << "{"
                 << "\"ok\":true,"
                 << "\"seq\":" << state->seq << ","
                 << "\"timestamp\":" << state->timestamp << ","
                 << "\"mode\":\"" << state->mode << "\","
                 << "\"paused\":" << (state->paused ? "true" : "false") << ","
                 << "\"cmd\":[" << state->command[0] << "," << state->command[1] << "," << state->command[2] << "],"
                 << "\"sim_time_s\":" << state->sim_time_s << ","
                 << "\"sim_steps\":" << state->sim_steps << ","
                 << "\"policy_steps\":" << state->policy_steps << ","
                 << "\"base\":[" << state->base_x << "," << state->base_y << "," << state->base_z << "],"
                 << "\"reset_pending\":" << (state->reset_requested ? "true" : "false") << ","
                 << "\"control_queue\":" << state->control_commands.size() << ","
                 << "\"viewer_event_queue\":" << state->viewer_events.size() << ","
                 << "\"http_reset_requests\":" << state->http_reset_requests << ","
                 << "\"http_viewer_events\":" << state->http_viewer_events << ","
                 << "\"http_control_commands\":" << state->http_control_commands << ","
                 << "\"push_force_steps\":" << state->push_force_steps << ","
                 << "\"push_impulse_applied\":" << (state->push_impulse_applied ? "true" : "false") << ","
                 << "\"mouse_perturb_steps\":" << state->mouse_perturb_steps
                 << "}\n";
            send_text(fd, 200, "OK", body.str(), "application/json");
        } else if (path == "/reset") {
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->reset_requested = true;
            }
            send_text(fd, 200, "OK", "{\"ok\":true,\"action\":\"reset\"}\n", "application/json");
        } else if (path == "/viewer-event") {
            ViewerHttpEvent event;
            event.type = query_value(query, "type");
            event.x = query_double(query, "x", 0.0);
            event.y = query_double(query, "y", 0.0);
            event.dx = query_double(query, "dx", 0.0);
            event.dy = query_double(query, "dy", 0.0);
            event.width = std::max(1.0, query_double(query, "width", 1.0));
            event.height = std::max(1.0, query_double(query, "height", 1.0));
            event.button = query_int(query, "button", 0);
            if (event.type.empty()) {
                send_text(fd, 400, "Bad Request", "{\"ok\":false,\"error\":\"missing type\"}\n", "application/json");
            } else {
                {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    if (state->viewer_events.size() > 128) state->viewer_events.erase(state->viewer_events.begin());
                    state->viewer_events.push_back(event);
                }
                send_text(fd, 200, "OK", "{\"ok\":true,\"action\":\"viewer-event\"}\n", "application/json");
            }
        } else if (path == "/control") {
            try {
                const ViewerControlCommand command = parse_control_command(query);
                {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    if (state->control_commands.size() > 128) {
                        state->control_commands.erase(state->control_commands.begin());
                    }
                    state->control_commands.push_back(command);
                }
                send_text(fd, 200, "OK", "{\"ok\":true,\"action\":\"control\"}\n", "application/json");
            } catch (const std::exception& e) {
                send_text(
                    fd,
                    400,
                    "Bad Request",
                    std::string("{\"ok\":false,\"error\":\"") + e.what() + "\"}\n",
                    "application/json");
            }
        } else if (path == "/frame.jpg") {
            std::vector<unsigned char> jpg;
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                jpg = state->latest_jpg;
            }
            if (jpg.empty()) send_text(fd, 503, "Service Unavailable", "no frame yet\n", "text/plain");
            else send_bytes(fd, 200, "OK", jpg, "image/jpeg");
        } else if (path == "/frame.png") {
            std::vector<unsigned char> png;
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                png = state->latest_png;
            }
            if (png.empty()) send_text(fd, 503, "Service Unavailable", "no frame yet\n", "text/plain");
            else send_bytes(fd, 200, "OK", png, "image/png");
        } else if (path == "/stream.mjpg") {
            std::string header =
                "HTTP/1.1 200 OK\r\n"
                "Age: 0\r\n"
                "Cache-Control: no-cache, private\r\n"
                "Pragma: no-cache\r\n"
                "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
            send_all(fd, header.data(), header.size());
            uint64_t last_seq = 0;
            while (true) {
                std::vector<unsigned char> jpg;
                uint64_t seq = 0;
                bool running = true;
                {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    running = state->running;
                    seq = state->seq;
                    if (seq != last_seq) jpg = state->latest_jpg;
                }
                if (!running) break;
                if (!jpg.empty()) {
                    std::string part =
                        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                        std::to_string(jpg.size()) + "\r\n\r\n";
                    send_all(fd, part.data(), part.size());
                    send_all(fd, reinterpret_cast<const char*>(jpg.data()), jpg.size());
                    send_all(fd, "\r\n", 2);
                    last_seq = seq;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
        } else {
            send_text(fd, 404, "Not Found", "not found\n", "text/plain");
        }
        close(fd);
    }

    void serve_loop()
    {
        listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd_ < 0) {
            std::perror("[CameraStream] socket");
            return;
        }
        int yes = 1;
        setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port_));
        addr.sin_addr.s_addr = inet_addr(host_.c_str());
        if (addr.sin_addr.s_addr == INADDR_NONE) addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            std::perror("[CameraStream] bind");
            close(listen_fd_);
            listen_fd_ = -1;
            return;
        }
        if (listen(listen_fd_, 8) < 0) {
            std::perror("[CameraStream] listen");
            close(listen_fd_);
            listen_fd_ = -1;
            return;
        }

        while (true) {
            {
                std::lock_guard<std::mutex> lock(state_->mutex);
                if (!state_->running) break;
            }
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(listen_fd_, &rfds);
            timeval tv{};
            tv.tv_sec = 0;
            tv.tv_usec = 200000;
            int ready = select(listen_fd_ + 1, &rfds, nullptr, nullptr, &tv);
            if (ready <= 0) continue;
            int fd = accept(listen_fd_, nullptr, nullptr);
            if (fd < 0) continue;
            std::thread(&CameraStreamServer::handle_client, fd, state_).detach();
        }
    }

    std::string host_;
    int port_{18080};
    int listen_fd_{-1};
    std::shared_ptr<CameraStreamState> state_;
    std::thread thread_;
};

#ifdef ENABLE_ROS2_CAMERA
class Ros2CameraPublisher {
public:
    Ros2CameraPublisher(
        const std::string& node_name,
        const std::string& topic_rgb,
        const std::string& topic_rgba,
        const std::string& frame_id,
        int qos_depth)
        : frame_id_(frame_id)
    {
        if (!rclcpp::ok()) {
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
            owns_init_ = true;
        }
        node_ = std::make_shared<rclcpp::Node>(node_name);
        rclcpp::QoS qos(static_cast<size_t>(std::max(1, qos_depth)));
        pub_rgb_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_rgb, qos);
        pub_rgba_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_rgba, qos);
        std::printf("[CameraROS2] publish %s and %s\n", topic_rgb.c_str(), topic_rgba.c_str());
    }

    ~Ros2CameraPublisher()
    {
        pub_rgb_.reset();
        pub_rgba_.reset();
        node_.reset();
        if (owns_init_ && rclcpp::ok()) rclcpp::shutdown();
    }

    void publish(
        const std::vector<unsigned char>& rgb,
        const std::vector<unsigned char>& rgba,
        int width,
        int height)
    {
        if (!node_ || rgb.empty() || rgba.empty()) return;
        auto rgb_msg = make_msg(rgb, width, height, 3, "rgb8");
        auto rgba_msg = make_msg(rgba, width, height, 4, "rgba8");
        pub_rgb_->publish(rgb_msg);
        pub_rgba_->publish(rgba_msg);
        rclcpp::spin_some(node_);
    }

private:
    sensor_msgs::msg::Image make_msg(
        const std::vector<unsigned char>& data,
        int width,
        int height,
        int channels,
        const std::string& encoding)
    {
        sensor_msgs::msg::Image msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = frame_id_;
        msg.height = static_cast<uint32_t>(height);
        msg.width = static_cast<uint32_t>(width);
        msg.encoding = encoding;
        msg.is_bigendian = 0;
        msg.step = static_cast<uint32_t>(width * channels);
        msg.data = data;
        return msg;
    }

    bool owns_init_{false};
    std::string frame_id_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgba_;
};
#endif

void usage(const char* argv0)
{
    std::printf(
        "Usage: %s [--config PATH] [--beyond-yaml PATH] [--mujoco-yaml PATH] [--xml PATH]\n"
        "          [--initial-pose-yaml PATH] [--loco] [--paused|--unpaused]\n"
        "          [--duration SEC] [--width N] [--height N]\n"
        "          [--summary-json PATH]\n"
        "          [--udp-control] [--udp-bind IP] [--udp-port N] [--udp-timeout-s SEC]\n"
        "          [--push-body NAME] [--push-force X,Y,Z] [--push-start SEC] [--push-duration SEC]\n"
        "          [--push-impulse X,Y,Z] [--push-impulse-time SEC]\n"
        "          [--camera-stream] [--camera-port N] [--camera-name NAME]\n"
        "          [--camera-ros2] [--ros2-topic-rgb TOPIC] [--ros2-topic-rgba TOPIC]\n"
        "\n"
        "Keys: L toggle loco, B Beyond/DANCE, T Track/SKILL, Space pause, R reset, F follow, X zero command,\n"
        "      W/S vx, Q/E vy, A/D wz, Esc close. Mouse drags move camera;\n"
        "      Shift+left/middle drag applies MuJoCo perturb force to the selected body.\n",
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
    if (iss >> extra) throw std::runtime_error("expected exactly three vec3 components");
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

Args parse_args(int argc, char** argv)
{
    Args args;
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--config") {
            args.config = need_value(i, argc, argv);
        } else if (arg == "--beyond-yaml") {
            args.beyond_yaml = need_value(i, argc, argv);
        } else if (arg == "--track-mimic-yaml") {
            args.track_mimic_yaml = need_value(i, argc, argv);
        } else if (arg == "--mujoco-yaml") {
            args.mujoco_yaml = need_value(i, argc, argv);
        } else if (arg == "--xml") {
            args.xml = need_value(i, argc, argv);
        } else if (arg == "--initial-pose-yaml") {
            args.initial_pose_yaml = need_value(i, argc, argv);
        } else if (arg == "--width") {
            args.width = std::atoi(need_value(i, argc, argv).c_str());
        } else if (arg == "--height") {
            args.height = std::atoi(need_value(i, argc, argv).c_str());
        } else if (arg == "--duration") {
            args.duration = std::atof(need_value(i, argc, argv).c_str());
        } else if (arg == "--sim-dt") {
            args.sim_dt = std::atof(need_value(i, argc, argv).c_str());
        } else if (arg == "--control-decimation") {
            args.control_decimation = std::atoi(need_value(i, argc, argv).c_str());
        } else if (arg == "--render-fps") {
            args.render_fps = std::atoi(need_value(i, argc, argv).c_str());
        } else if (arg == "--loco") {
            args.start_loco = true;
        } else if (arg == "--paused") {
            args.paused = true;
        } else if (arg == "--unpaused") {
            args.paused = false;
        } else if (arg == "--no-realtime") {
            args.realtime = false;
        } else if (arg == "--no-follow") {
            args.follow = false;
        } else if (arg == "--summary-json") {
            args.summary_json = need_value(i, argc, argv);
        } else if (arg == "--udp-control") {
            args.udp_control = true;
        } else if (arg == "--udp-bind") {
            args.udp_bind = need_value(i, argc, argv);
        } else if (arg == "--udp-port") {
            args.udp_port = std::atoi(need_value(i, argc, argv).c_str());
        } else if (arg == "--udp-timeout-s") {
            args.udp_timeout_s = std::atof(need_value(i, argc, argv).c_str());
        } else if (arg == "--push-body") {
            args.push_body = need_value(i, argc, argv);
        } else if (arg == "--push-force") {
            args.push_force = parse_vec3(need_value(i, argc, argv));
        } else if (arg == "--push-start") {
            args.push_start_s = std::atof(need_value(i, argc, argv).c_str());
        } else if (arg == "--push-duration") {
            args.push_duration_s = std::atof(need_value(i, argc, argv).c_str());
        } else if (arg == "--push-impulse") {
            args.push_impulse = parse_vec3(need_value(i, argc, argv));
        } else if (arg == "--push-impulse-time") {
            args.push_impulse_time_s = std::atof(need_value(i, argc, argv).c_str());
        } else if (arg == "--camera-stream") {
            args.camera_stream = true;
            args.camera_stream_set = true;
        } else if (arg == "--no-camera-stream") {
            args.camera_stream = false;
            args.camera_stream_set = true;
        } else if (arg == "--camera-name") {
            args.camera_name = need_value(i, argc, argv);
            args.camera_name_set = true;
        } else if (arg == "--camera-host") {
            args.camera_host = need_value(i, argc, argv);
            args.camera_host_set = true;
        } else if (arg == "--camera-port") {
            args.camera_port = std::atoi(need_value(i, argc, argv).c_str());
            args.camera_port_set = true;
        } else if (arg == "--camera-width") {
            args.camera_width = std::atoi(need_value(i, argc, argv).c_str());
            args.camera_width_set = true;
        } else if (arg == "--camera-height") {
            args.camera_height = std::atoi(need_value(i, argc, argv).c_str());
            args.camera_height_set = true;
        } else if (arg == "--camera-fps") {
            args.camera_fps = std::atoi(need_value(i, argc, argv).c_str());
            args.camera_fps_set = true;
        } else if (arg == "--jpeg-quality") {
            args.jpeg_quality = std::atoi(need_value(i, argc, argv).c_str());
            args.jpeg_quality_set = true;
        } else if (arg == "--camera-ros2") {
            args.camera_ros2 = true;
            args.camera_ros2_set = true;
        } else if (arg == "--no-camera-ros2") {
            args.camera_ros2 = false;
            args.camera_ros2_set = true;
        } else if (arg == "--ros2-topic-rgb") {
            args.ros2_topic_rgb = need_value(i, argc, argv);
            args.ros2_topic_rgb_set = true;
        } else if (arg == "--ros2-topic-rgba") {
            args.ros2_topic_rgba = need_value(i, argc, argv);
            args.ros2_topic_rgba_set = true;
        } else if (arg == "--ros2-frame-id") {
            args.ros2_frame_id = need_value(i, argc, argv);
            args.ros2_frame_id_set = true;
        } else if (arg == "--ros2-node-name") {
            args.ros2_node_name = need_value(i, argc, argv);
            args.ros2_node_name_set = true;
        } else if (arg == "--ros2-qos-depth") {
            args.ros2_qos_depth = std::atoi(need_value(i, argc, argv).c_str());
            args.ros2_qos_depth_set = true;
        } else if (arg == "-h" || arg == "--help") {
            usage(argv[0]);
            std::exit(0);
        } else {
            std::fprintf(stderr, "Unknown arg: %s\n", arg.c_str());
            usage(argv[0]);
            std::exit(2);
        }
    }
    args.width = std::max(320, args.width);
    args.height = std::max(240, args.height);
    args.control_decimation = std::max(1, args.control_decimation);
    args.render_fps = std::max(1, args.render_fps);
    args.udp_port = std::clamp(args.udp_port, 1, 65535);
    args.udp_timeout_s = std::clamp(args.udp_timeout_s, 0.02, 10.0);
    args.camera_port = std::max(1, args.camera_port);
    args.camera_width = std::max(64, args.camera_width);
    args.camera_height = std::max(64, args.camera_height);
    args.camera_fps = std::max(1, args.camera_fps);
    args.jpeg_quality = std::clamp(args.jpeg_quality, 1, 100);
    args.ros2_qos_depth = std::max(1, args.ros2_qos_depth);
    if (args.sim_dt <= 0.0) args.sim_dt = 0.002;
    args.push_start_s = std::max(0.0, args.push_start_s);
    args.push_duration_s = std::max(0.0, args.push_duration_s);
    if (args.push_impulse_time_s < 0.0) args.push_impulse_time_s = args.push_start_s;
    if (has_vec3(args.push_force) && args.push_duration_s <= 0.0) {
        throw std::runtime_error("--push-duration must be > 0 when --push-force is nonzero");
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
    fs::path source = fs::absolute(fs::path(__FILE__));
    for (fs::path p = source.parent_path(); !p.empty(); p = p.parent_path()) {
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
    const fs::path yaml_path = resolve_path(root, args.mujoco_yaml);
    if (!fs::exists(yaml_path)) return;
    YAML::Node cfg = YAML::LoadFile(yaml_path.string());
    if (args.xml.empty() && cfg["xml_path"]) args.xml = cfg["xml_path"].as<std::string>();
    if (args.initial_pose_yaml.empty() && cfg["initial_pose_yaml"]) {
        args.initial_pose_yaml = cfg["initial_pose_yaml"].as<std::string>();
    }
    if (cfg["simulation_dt"]) args.sim_dt = cfg["simulation_dt"].as<double>();
    if (cfg["control_decimation"]) args.control_decimation = cfg["control_decimation"].as<int>();
    if (cfg["render_fps"]) args.render_fps = cfg["render_fps"].as<int>();
    if (cfg["initial_base_height"]) args.initial_base_height = cfg["initial_base_height"].as<double>();
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
    if (cfg["head_camera_stream"]) {
        YAML::Node cam = cfg["head_camera_stream"];
        if (!args.camera_stream_set && cam["enable"]) args.camera_stream = cam["enable"].as<bool>();
        if (!args.camera_name_set && cam["camera_name"]) args.camera_name = cam["camera_name"].as<std::string>();
        if (!args.camera_host_set && cam["host"]) args.camera_host = cam["host"].as<std::string>();
        if (!args.camera_port_set && cam["port"]) args.camera_port = cam["port"].as<int>();
        if (!args.camera_width_set && cam["width"]) args.camera_width = cam["width"].as<int>();
        if (!args.camera_height_set && cam["height"]) args.camera_height = cam["height"].as<int>();
        if (!args.camera_fps_set && cam["fps"]) args.camera_fps = cam["fps"].as<int>();
        if (!args.jpeg_quality_set && cam["jpeg_quality"]) args.jpeg_quality = cam["jpeg_quality"].as<int>();
    }
    if (cfg["head_camera_ros2"]) {
        YAML::Node ros = cfg["head_camera_ros2"];
        if (!args.camera_ros2_set && ros["enable"]) args.camera_ros2 = ros["enable"].as<bool>();
        if (!args.camera_name_set && ros["camera_name"]) args.camera_name = ros["camera_name"].as<std::string>();
        if (!args.camera_width_set && ros["width"]) args.camera_width = ros["width"].as<int>();
        if (!args.camera_height_set && ros["height"]) args.camera_height = ros["height"].as<int>();
        if (!args.camera_fps_set && ros["fps"]) args.camera_fps = ros["fps"].as<int>();
        if (!args.ros2_topic_rgb_set && ros["topic_rgb"]) args.ros2_topic_rgb = ros["topic_rgb"].as<std::string>();
        if (!args.ros2_topic_rgba_set && ros["topic_rgba"]) args.ros2_topic_rgba = ros["topic_rgba"].as<std::string>();
        if (!args.ros2_frame_id_set && ros["frame_id"]) args.ros2_frame_id = ros["frame_id"].as<std::string>();
        if (!args.ros2_node_name_set && ros["node_name"]) args.ros2_node_name = ros["node_name"].as<std::string>();
        if (!args.ros2_qos_depth_set && ros["qos_depth"]) args.ros2_qos_depth = ros["qos_depth"].as<int>();
    }
}

std::vector<float> read_fvec(const YAML::Node& cfg, const char* key)
{
    std::vector<float> out;
    if (!cfg[key]) return out;
    for (const auto& v : cfg[key]) out.push_back(v.as<float>());
    return out;
}

std::vector<int> read_ivec(const YAML::Node& cfg, const char* key)
{
    std::vector<int> out;
    if (!cfg[key]) return out;
    for (const auto& v : cfg[key]) out.push_back(v.as<int>());
    return out;
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

magicbot_loco::JointArray mapped_lab_vector(
    const std::vector<float>& lab,
    const std::vector<int>& lab_to_mj)
{
    magicbot_loco::JointArray out{};
    for (size_t lab_idx = 0; lab_idx < lab.size() && lab_idx < lab_to_mj.size(); ++lab_idx) {
        const int mj_idx = lab_to_mj[lab_idx];
        if (mj_idx >= 0 && mj_idx < magicbot_loco::kNumJoints) out[static_cast<size_t>(mj_idx)] = lab[lab_idx];
    }
    return out;
}

bool load_initial_pose(
    const fs::path& root,
    const std::string& yaml_raw,
    magicbot_loco::JointArray& q,
    magicbot_loco::JointArray& kp,
    magicbot_loco::JointArray& kd,
    magicbot_loco::JointArray& tau_limit)
{
    if (yaml_raw.empty()) return false;
    const fs::path path = resolve_path(root, yaml_raw);
    if (!fs::exists(path)) return false;
    YAML::Node cfg = YAML::LoadFile(path.string());
    const auto lab_to_mj = read_ivec(cfg, "mj2lab");
    const auto q_lab = read_fvec(cfg, "default_angles_lab");
    const auto kp_lab = read_fvec(cfg, "kp_lab");
    const auto kd_lab = read_fvec(cfg, "kd_lab");
    const auto tau_lab = read_fvec(cfg, "tau_limit");
    if (lab_to_mj.empty() || q_lab.empty()) return false;
    q = mapped_lab_vector(q_lab, lab_to_mj);
    if (!kp_lab.empty()) kp = mapped_lab_vector(kp_lab, lab_to_mj);
    if (!kd_lab.empty()) kd = mapped_lab_vector(kd_lab, lab_to_mj);
    if (!tau_lab.empty()) tau_limit = mapped_lab_vector(tau_lab, lab_to_mj);
    return true;
}

void reset_sim(
    mjModel* model,
    mjData* data,
    const std::vector<int>& qpos_idx,
    double base_height,
    const magicbot_loco::JointArray& stand_q)
{
    mj_resetData(model, data);
    data->qpos[0] = 0.0;
    data->qpos[1] = 0.0;
    data->qpos[2] = base_height;
    data->qpos[3] = 1.0;
    data->qpos[4] = 0.0;
    data->qpos[5] = 0.0;
    data->qpos[6] = 0.0;
    for (int i = 0; i < model->nu && i < magicbot_loco::kNumJoints; ++i) {
        data->qpos[7 + qpos_idx[i]] = stand_q[static_cast<size_t>(i)];
    }
    for (int i = 0; i < model->nv; ++i) data->qvel[i] = 0.0;
    mj_forward(model, data);
}

magicbot_loco::ControlMode control_mode_for_fsm_state(FSMStateName state)
{
    switch (state) {
    case FSMStateName::PASSIVE:
        return magicbot_loco::ControlMode::Passive;
    case FSMStateName::FIXEDPOSE:
        return magicbot_loco::ControlMode::Stand;
    case FSMStateName::LOCOMODE:
    case FSMStateName::SKILL_COOLDOWN:
        return magicbot_loco::ControlMode::Loco;
    case FSMStateName::SKILL_BEYOND_MIMIC:
    case FSMStateName::SKILL_DANCE:
        return magicbot_loco::ControlMode::Dance;
    case FSMStateName::SKILL_TRACK_MIMIC:
        return magicbot_loco::ControlMode::Skill;
    default:
        return magicbot_loco::ControlMode::Skill;
    }
}

const char* viewer_mode_label(bool loco, bool passive, bool dance, bool skill, bool final_damping);

std::string body_name(const mjModel* model, int body_id)
{
    if (body_id <= 0) return "";
    const char* raw = mj_id2name(model, mjOBJ_BODY, body_id);
    return raw ? std::string(raw) : std::string();
}

std::string json_escape(const std::string& value)
{
    std::ostringstream out;
    for (char c : value) {
        switch (c) {
        case '\\':
            out << "\\\\";
            break;
        case '"':
            out << "\\\"";
            break;
        case '\n':
            out << "\\n";
            break;
        case '\r':
            out << "\\r";
            break;
        case '\t':
            out << "\\t";
            break;
        default:
            out << c;
            break;
        }
    }
    return out.str();
}

struct PushStepResult {
    bool force_active{false};
    bool impulse_applied{false};
};

PushStepResult apply_push_disturbance(
    const Args& args,
    mjModel* model,
    mjData* data,
    int push_body_id,
    bool& impulse_done)
{
    PushStepResult result;
    if (push_body_id < 0 || data->xfrc_applied == nullptr) return result;

    const double sim_time = data->time;
    const double dt = std::max(1e-9, model->opt.timestep);
    double* xfrc = data->xfrc_applied + 6 * push_body_id;

    const bool force_active =
        has_vec3(args.push_force) &&
        sim_time + 0.5 * dt >= args.push_start_s &&
        sim_time < args.push_start_s + args.push_duration_s;
    if (force_active) {
        for (int i = 0; i < 3; ++i) xfrc[i] += args.push_force[static_cast<size_t>(i)];
        result.force_active = true;
    }

    const bool impulse_active =
        has_vec3(args.push_impulse) &&
        !impulse_done &&
        sim_time + 0.5 * dt >= args.push_impulse_time_s;
    if (impulse_active) {
        for (int i = 0; i < 3; ++i) xfrc[i] += args.push_impulse[static_cast<size_t>(i)] / dt;
        impulse_done = true;
        result.impulse_applied = true;
    }
    return result;
}

std::string viewer_summary_json(
    const Args& args,
    const mjModel* model,
    const mjData* data,
    const ViewerStats& stats,
    int sim_step,
    int policy_step,
    bool loco_active,
    bool passive_active,
    bool dance_active,
    bool skill_active,
    bool final_damping_active,
    bool paused,
    double wall_s,
    int push_body_id)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(6)
        << "{"
        << "\"mode\":\"" << viewer_mode_label(loco_active, passive_active, dance_active, skill_active, final_damping_active) << "\","
        << "\"paused\":" << (paused ? "true" : "false") << ","
        << "\"wall_s\":" << wall_s << ","
        << "\"sim_time_s\":" << data->time << ","
        << "\"sim_steps\":" << sim_step << ","
        << "\"policy_steps\":" << policy_step << ","
        << "\"base_x\":" << data->qpos[0] << ","
        << "\"base_y\":" << data->qpos[1] << ","
        << "\"base_z\":" << data->qpos[2] << ","
        << "\"min_base_height\":" << (std::isfinite(stats.min_base_height) ? stats.min_base_height : data->qpos[2]) << ","
        << "\"max_root_xy_drift\":" << stats.max_root_xy_drift << ","
        << "\"max_gravity_xy\":" << stats.max_gravity_xy << ","
        << "\"push_body\":\"" << json_escape(args.push_body) << "\","
        << "\"push_body_id\":" << push_body_id << ","
        << "\"push_body_resolved\":\"" << json_escape(body_name(model, push_body_id)) << "\","
        << "\"push_enabled\":" << (has_vec3(args.push_force) || has_vec3(args.push_impulse) ? "true" : "false") << ","
        << "\"push_start_s\":" << args.push_start_s << ","
        << "\"push_duration_s\":" << args.push_duration_s << ","
        << "\"push_impulse_time_s\":" << args.push_impulse_time_s << ","
        << "\"push_force_norm\":" << vec3_norm(args.push_force) << ","
        << "\"push_impulse_norm\":" << vec3_norm(args.push_impulse) << ","
        << "\"push_force_steps\":" << stats.push_force_steps << ","
        << "\"push_impulse_applied\":" << (stats.push_impulse_applied ? "true" : "false") << ","
        << "\"mouse_perturb_steps\":" << stats.mouse_perturb_steps << ","
        << "\"http_reset_requests\":" << stats.http_reset_requests << ","
        << "\"http_viewer_events\":" << stats.http_viewer_events << ","
        << "\"http_control_commands\":" << stats.http_control_commands << ","
        << "\"last_perturb_body\":" << stats.last_perturb_body << ","
        << "\"last_perturb_body_name\":\"" << json_escape(stats.last_perturb_body_name) << "\""
        << "}";
    return out.str();
}

XGlWindow create_window(int width, int height)
{
    XGlWindow out;
    out.width = width;
    out.height = height;
    out.display = XOpenDisplay(nullptr);
    if (!out.display) throw std::runtime_error("XOpenDisplay failed; check DISPLAY");

    int attrs[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE, 24, GLX_STENCIL_SIZE, 8, 0};
    XVisualInfo* vi = glXChooseVisual(out.display, DefaultScreen(out.display), attrs);
    if (!vi) throw std::runtime_error("glXChooseVisual failed");

    out.colormap = XCreateColormap(
        out.display,
        RootWindow(out.display, vi->screen),
        vi->visual,
        AllocNone);

    XSetWindowAttributes swa{};
    swa.colormap = out.colormap;
    swa.event_mask =
        ExposureMask | KeyPressMask | ButtonPressMask | ButtonReleaseMask |
        PointerMotionMask | StructureNotifyMask;

    out.window = XCreateWindow(
        out.display,
        RootWindow(out.display, vi->screen),
        0,
        0,
        static_cast<unsigned int>(width),
        static_cast<unsigned int>(height),
        0,
        vi->depth,
        InputOutput,
        vi->visual,
        CWColormap | CWEventMask,
        &swa);
    XStoreName(out.display, out.window, "MagicBot Z1 Native MuJoCo Loco Viewer");
    out.wm_delete = XInternAtom(out.display, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(out.display, out.window, &out.wm_delete, 1);
    XMapWindow(out.display, out.window);

    out.context = glXCreateContext(out.display, vi, nullptr, GL_TRUE);
    XFree(vi);
    if (!out.context) throw std::runtime_error("glXCreateContext failed");
    glXMakeCurrent(out.display, out.window, out.context);
    glEnable(GL_DEPTH_TEST);
    return out;
}

void destroy_window(XGlWindow& win)
{
    if (!win.display) return;
    if (win.context) {
        glXMakeCurrent(win.display, 0, nullptr);
        glXDestroyContext(win.display, win.context);
    }
    if (win.window) XDestroyWindow(win.display, win.window);
    if (win.colormap) XFreeColormap(win.display, win.colormap);
    XCloseDisplay(win.display);
    win = XGlWindow{};
}

float clamp_cmd(float v) { return std::clamp(v, -1.0f, 1.0f); }

void apply_viewer_text_action(
    magicbot_loco::TextControlAction action,
    std::array<float, 3>& cmd,
    bool& loco_active,
    bool& passive_active,
    bool& dance_active,
    bool& skill_active,
    bool& final_damping_active,
    bool dance_enabled,
    bool skill_enabled,
    bool& paused,
    bool& running,
    bool& reset_requested)
{
    const magicbot_loco::TextControlActionEffect effect = magicbot_loco::text_control_action_effect(action);
    if (effect.mode_requested && effect.mode == magicbot_loco::ControlMode::Dance && !dance_enabled) {
        std::fprintf(stderr, "[Viewer] DANCE ignored; start with --beyond-yaml PATH to enable BeyondMimic\n");
        return;
    }
    if (effect.mode_requested && effect.mode == magicbot_loco::ControlMode::Skill && !skill_enabled) {
        std::fprintf(stderr, "[Viewer] SKILL ignored; start with --track-mimic-yaml PATH to enable BeyondMimic trajectory/TrackMimic\n");
        return;
    }

    if (effect.zero_command) {
        cmd = {0.0f, 0.0f, 0.0f};
    }
    if (effect.pause) {
        paused = true;
    }
    if (effect.unpause) {
        paused = false;
    }
    if (effect.stop) {
        running = false;
    }
    if (effect.reset_stand) {
        passive_active = false;
        dance_active = false;
        skill_active = false;
        final_damping_active = false;
        reset_requested = true;
    }
    if (effect.toggle_loco) {
        if (loco_active) {
            loco_active = false;
            cmd = {0.0f, 0.0f, 0.0f};
        } else {
            reset_requested = true;
            loco_active = true;
        }
        passive_active = false;
        dance_active = false;
        skill_active = false;
        final_damping_active = false;
    }
    if (!effect.mode_requested) {
        return;
    }

    const magicbot_loco::ControlMode mode = effect.mode;
    if (mode == magicbot_loco::ControlMode::Loco &&
        (!loco_active || passive_active || dance_active || skill_active || final_damping_active)) {
        reset_requested = true;
    }
    if (mode == magicbot_loco::ControlMode::Stand) {
        reset_requested = true;
    }
    final_damping_active = mode == magicbot_loco::ControlMode::FinalDamping;
    passive_active = mode == magicbot_loco::ControlMode::Passive;
    loco_active = mode == magicbot_loco::ControlMode::Loco;
    dance_active = mode == magicbot_loco::ControlMode::Dance;
    skill_active = mode == magicbot_loco::ControlMode::Skill;
}

class ViewerUdpCommandInput {
public:
    explicit ViewerUdpCommandInput(const Args& args, bool dance_enabled, bool skill_enabled)
        : args_(args),
          dance_enabled_(dance_enabled),
          skill_enabled_(skill_enabled)
    {
        fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) throw std::runtime_error(std::string("viewer udp socket failed: ") + std::strerror(errno));

        const int flags = fcntl(fd_, F_GETFL, 0);
        if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) != 0) {
            throw std::runtime_error(std::string("viewer udp fcntl failed: ") + std::strerror(errno));
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
                "viewer udp bind " + args_.udp_bind + ":" + std::to_string(args_.udp_port) +
                " failed: " + std::strerror(errno));
        }
    }

    ~ViewerUdpCommandInput()
    {
        if (fd_ >= 0) close(fd_);
    }

    bool poll(
        std::array<float, 3>& cmd,
        bool& loco_active,
        bool& passive_active,
        bool& dance_active,
        bool& skill_active,
        bool& final_damping_active,
        bool& paused,
        bool& running,
        bool& reset_requested)
    {
        bool changed = false;
        while (true) {
            char buffer[512]{};
            sockaddr_in src{};
            socklen_t src_len = sizeof(src);
            const ssize_t n = recvfrom(
                fd_, buffer, sizeof(buffer) - 1, 0,
                reinterpret_cast<sockaddr*>(&src), &src_len);
            if (n > 0) {
                buffer[n] = '\0';
                const auto old_cmd = cmd;
                const bool old_loco_active = loco_active;
                const bool old_passive_active = passive_active;
                const bool old_dance_active = dance_active;
                const bool old_skill_active = skill_active;
                const bool old_final_damping_active = final_damping_active;
                const bool old_paused = paused;
                const bool old_running = running;
                const bool old_reset_requested = reset_requested;
                handle_message(
                    std::string(buffer, static_cast<size_t>(n)),
                    cmd,
                    loco_active,
                    passive_active,
                    dance_active,
                    skill_active,
                    final_damping_active,
                    paused,
                    running,
                    reset_requested);
                have_packet_ = true;
                last_packet_t_ = Clock::now();
                changed = changed || old_cmd != cmd ||
                          old_loco_active != loco_active ||
                          old_passive_active != passive_active ||
                          old_dance_active != dance_active ||
                          old_skill_active != skill_active ||
                          old_final_damping_active != final_damping_active ||
                          old_paused != paused ||
                          old_running != running || old_reset_requested != reset_requested;
                continue;
            }
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                throw std::runtime_error(std::string("viewer udp recvfrom failed: ") + std::strerror(errno));
            }
            break;
        }

        if (have_packet_ && std::chrono::duration<double>(Clock::now() - last_packet_t_).count() > args_.udp_timeout_s) {
            if (cmd != std::array<float, 3>{0.0f, 0.0f, 0.0f}) {
                cmd = {0.0f, 0.0f, 0.0f};
                changed = true;
            }
        }
        return changed;
    }

private:
    void handle_message(
        std::string message,
        std::array<float, 3>& cmd,
        bool& loco_active,
        bool& passive_active,
        bool& dance_active,
        bool& skill_active,
        bool& final_damping_active,
        bool& paused,
        bool& running,
        bool& reset_requested)
    {
        for (const magicbot_loco::TextControlOperation& op :
             magicbot_loco::parse_text_control_operations(std::move(message))) {
            if (op.type == magicbot_loco::TextControlOperation::Type::Velocity) {
                if (op.axis >= 0 && op.axis < 3) {
                    cmd[static_cast<size_t>(op.axis)] = op.value;
                }
            } else {
                apply_viewer_text_action(
                    op.action,
                    cmd,
                    loco_active,
                    passive_active,
                    dance_active,
                    skill_active,
                    final_damping_active,
                    dance_enabled_,
                    skill_enabled_,
                    paused,
                    running,
                    reset_requested);
            }
        }
    }

    const Args& args_;
    bool dance_enabled_{false};
    bool skill_enabled_{false};
    int fd_{-1};
    bool have_packet_{false};
    Clock::time_point last_packet_t_{};
};

const char* viewer_mode_label(bool loco, bool passive, bool dance, bool skill, bool final_damping)
{
    if (final_damping) return "FINAL_DAMPING";
    if (passive) return "PASSIVE";
    if (dance) return "DANCE";
    if (skill) return "SKILL";
    return loco ? "LOCO" : "STAND";
}

magicbot_loco::ControlMode viewer_control_mode(bool loco, bool passive, bool dance, bool skill, bool final_damping)
{
    if (final_damping) return magicbot_loco::ControlMode::FinalDamping;
    if (passive) return magicbot_loco::ControlMode::Passive;
    if (dance) return magicbot_loco::ControlMode::Dance;
    if (skill) return magicbot_loco::ControlMode::Skill;
    return loco ? magicbot_loco::ControlMode::Loco : magicbot_loco::ControlMode::Stand;
}

magicbot_loco::ModeRequest viewer_mode_request(bool loco, bool passive, bool dance, bool skill, bool final_damping)
{
    return magicbot_loco::mode_request_for_control_mode(
        viewer_control_mode(loco, passive, dance, skill, final_damping));
}

void sync_viewer_mode_flags(
    magicbot_loco::ControlMode mode,
    bool& loco,
    bool& passive,
    bool& dance,
    bool& skill,
    bool& final_damping)
{
    final_damping = mode == magicbot_loco::ControlMode::FinalDamping;
    passive = mode == magicbot_loco::ControlMode::Passive;
    loco = mode == magicbot_loco::ControlMode::Loco;
    dance = mode == magicbot_loco::ControlMode::Dance;
    skill = mode == magicbot_loco::ControlMode::Skill;
}

void print_cmd(
    const std::array<float, 3>& cmd,
    bool loco,
    bool passive,
    bool dance,
    bool skill,
    bool final_damping,
    bool paused)
{
    std::printf(
        "[Viewer] mode=%s paused=%s cmd(vx,vy,wz)=[%.2f %.2f %.2f]\n",
        viewer_mode_label(loco, passive, dance, skill, final_damping),
        paused ? "yes" : "no",
        cmd[0],
        cmd[1],
        cmd[2]);
}

void apply_viewer_control_command(
    const ViewerControlCommand& control,
    std::array<float, 3>& cmd,
    bool& loco_active,
    bool& passive_active,
    bool& dance_active,
    bool& skill_active,
    bool& final_damping_active,
    bool dance_enabled,
    bool skill_enabled,
    bool& paused,
    bool& running,
    bool& reset_requested)
{
    if (control.has_vx) cmd[0] = clamp_cmd(control.velocity[0]);
    if (control.has_vy) cmd[1] = clamp_cmd(control.velocity[1]);
    if (control.has_wz) cmd[2] = clamp_cmd(control.velocity[2]);
    if (control.has_pause) paused = control.paused;
    if (!control.has_mode) return;

    apply_viewer_text_action(
        control.action,
        cmd,
        loco_active,
        passive_active,
        dance_active,
        skill_active,
        final_damping_active,
        dance_enabled,
        skill_enabled,
        paused,
        running,
        reset_requested);
}

void handle_key(
    KeySym sym,
    bool& running,
    bool& loco_active,
    bool& passive_active,
    bool& dance_active,
    bool& skill_active,
    bool& final_damping_active,
    bool dance_enabled,
    bool skill_enabled,
    bool& paused,
    bool& follow,
    bool& reset_requested,
    std::array<float, 3>& cmd)
{
    constexpr float dv = 0.1f;
    switch (sym) {
    case XK_Escape:
        running = false;
        return;
    case XK_space:
        paused = !paused;
        break;
    case XK_l:
    case XK_L:
        loco_active = !loco_active;
        passive_active = false;
        dance_active = false;
        skill_active = false;
        final_damping_active = false;
        break;
    case XK_m:
    case XK_M:
        cmd = {0.0f, 0.0f, 0.0f};
        loco_active = false;
        passive_active = true;
        dance_active = false;
        skill_active = false;
        final_damping_active = false;
        paused = false;
        break;
    case XK_n:
    case XK_N:
        cmd = {0.0f, 0.0f, 0.0f};
        loco_active = false;
        passive_active = false;
        dance_active = false;
        skill_active = false;
        final_damping_active = true;
        paused = false;
        break;
    case XK_b:
    case XK_B:
        if (!dance_enabled) {
            std::fprintf(stderr, "[Viewer] DANCE ignored; start with --beyond-yaml PATH to enable BeyondMimic\n");
            return;
        }
        loco_active = false;
        passive_active = false;
        dance_active = !dance_active;
        skill_active = false;
        final_damping_active = false;
        if (dance_active) paused = false;
        break;
    case XK_t:
    case XK_T:
        if (!skill_enabled) {
            std::fprintf(stderr, "[Viewer] SKILL ignored; start with --track-mimic-yaml PATH to enable BeyondMimic trajectory/TrackMimic\n");
            return;
        }
        loco_active = false;
        passive_active = false;
        dance_active = false;
        skill_active = !skill_active;
        final_damping_active = false;
        if (skill_active) paused = false;
        break;
    case XK_r:
    case XK_R:
        reset_requested = true;
        passive_active = false;
        dance_active = false;
        skill_active = false;
        final_damping_active = false;
        break;
    case XK_f:
    case XK_F:
        follow = !follow;
        break;
    case XK_x:
    case XK_X:
        cmd = {0.0f, 0.0f, 0.0f};
        break;
    case XK_w:
    case XK_W:
    case XK_Up:
        cmd[0] = clamp_cmd(cmd[0] + dv);
        break;
    case XK_s:
    case XK_S:
    case XK_Down:
        cmd[0] = clamp_cmd(cmd[0] - dv);
        break;
    case XK_q:
    case XK_Q:
        cmd[1] = clamp_cmd(cmd[1] + dv);
        break;
    case XK_e:
    case XK_E:
        cmd[1] = clamp_cmd(cmd[1] - dv);
        break;
    case XK_a:
    case XK_A:
    case XK_Left:
        cmd[2] = clamp_cmd(cmd[2] + dv);
        break;
    case XK_d:
    case XK_D:
    case XK_Right:
        cmd[2] = clamp_cmd(cmd[2] - dv);
        break;
    default:
        return;
    }
    print_cmd(cmd, loco_active, passive_active, dance_active, skill_active, final_damping_active, paused);
}

void process_events(
    XGlWindow& win,
    mjModel* model,
    mjData* data,
    mjvScene* scene,
    mjvOption* opt,
    mjvPerturb* perturb,
    mjvCamera* cam,
    MouseState& mouse,
    ViewerStats& stats,
    bool& running,
    bool& loco_active,
    bool& passive_active,
    bool& dance_active,
    bool& skill_active,
    bool& final_damping_active,
    bool dance_enabled,
    bool skill_enabled,
    bool& paused,
    bool& follow,
    bool& reset_requested,
    std::array<float, 3>& cmd)
{
    while (XPending(win.display) > 0) {
        XEvent event;
        XNextEvent(win.display, &event);
        switch (event.type) {
        case ClientMessage:
            if (static_cast<Atom>(event.xclient.data.l[0]) == win.wm_delete) running = false;
            break;
        case ConfigureNotify:
            win.width = std::max(1, event.xconfigure.width);
            win.height = std::max(1, event.xconfigure.height);
            break;
        case KeyPress: {
            KeySym sym = XLookupKeysym(&event.xkey, 0);
            handle_key(
                sym,
                running,
                loco_active,
                passive_active,
                dance_active,
                skill_active,
                final_damping_active,
                dance_enabled,
                skill_enabled,
                paused,
                follow,
                reset_requested,
                cmd);
            break;
        }
        case ButtonPress:
            mouse.last_x = event.xbutton.x;
            mouse.last_y = event.xbutton.y;
            if ((event.xbutton.state & ShiftMask) &&
                (event.xbutton.button == Button1 || event.xbutton.button == Button2)) {
                const mjtNum aspect = static_cast<mjtNum>(win.width) / std::max(1, win.height);
                const mjtNum relx = static_cast<mjtNum>(event.xbutton.x) / std::max(1, win.width);
                const mjtNum rely = static_cast<mjtNum>(win.height - event.xbutton.y) / std::max(1, win.height);
                mjtNum selpnt[3] = {0, 0, 0};
                int selgeom = -1;
                int selflex = -1;
                int selskin = -1;
                const int selbody = mjv_select(
                    model,
                    data,
                    opt,
                    aspect,
                    relx,
                    rely,
                    scene,
                    selpnt,
                    &selgeom,
                    &selflex,
                    &selskin);
                if (selbody > 0) {
                    perturb->select = selbody;
                    perturb->flexselect = selflex;
                    perturb->skinselect = selskin;
                    mjtNum tmp[3];
                    mju_sub3(tmp, selpnt, data->xpos + 3 * perturb->select);
                    mju_mulMatTVec(perturb->localpos, data->xmat + 9 * perturb->select, tmp, 3, 3);
                    mjv_initPerturb(model, data, scene, perturb);
                    perturb->active = mjPERT_TRANSLATE;
                    mouse.perturb = true;
                    mouse.perturb_button = event.xbutton.button;
                    stats.last_perturb_body = selbody;
                    stats.last_perturb_body_name = body_name(model, selbody);
                    std::printf("[ViewerPerturb] selected body=%d name=%s geom=%d\n",
                                selbody,
                                stats.last_perturb_body_name.c_str(),
                                selgeom);
                } else {
                    perturb->select = 0;
                    perturb->active = 0;
                    mouse.perturb = false;
                    mouse.perturb_button = 0;
                }
                break;
            }
            if (event.xbutton.button == Button1) mouse.left = true;
            if (event.xbutton.button == Button2) mouse.middle = true;
            if (event.xbutton.button == Button3) mouse.right = true;
            if (event.xbutton.button == Button4) {
                mjv_moveCamera(model, mjMOUSE_ZOOM, 0.0, -0.05, scene, cam);
            }
            if (event.xbutton.button == Button5) {
                mjv_moveCamera(model, mjMOUSE_ZOOM, 0.0, 0.05, scene, cam);
            }
            break;
        case ButtonRelease:
            if (mouse.perturb && event.xbutton.button == mouse.perturb_button) {
                perturb->active = 0;
                mouse.perturb = false;
                mouse.perturb_button = 0;
                break;
            }
            if (event.xbutton.button == Button1) mouse.left = false;
            if (event.xbutton.button == Button2) mouse.middle = false;
            if (event.xbutton.button == Button3) mouse.right = false;
            break;
        case MotionNotify: {
            const double dx = static_cast<double>(event.xmotion.x - mouse.last_x) / std::max(1, win.height);
            const double dy = static_cast<double>(event.xmotion.y - mouse.last_y) / std::max(1, win.height);
            mouse.last_x = event.xmotion.x;
            mouse.last_y = event.xmotion.y;
            if (mouse.perturb && perturb->active) {
                const int action = mouse.perturb_button == Button2 ? mjMOUSE_MOVE_V : mjMOUSE_MOVE_H;
                mjv_movePerturb(model, data, action, dx, -dy, scene, perturb);
            } else if (mouse.left) {
                mjv_moveCamera(model, mjMOUSE_ROTATE_H, dx, dy, scene, cam);
            } else if (mouse.right) {
                mjv_moveCamera(model, mjMOUSE_MOVE_H, dx, dy, scene, cam);
            } else if (mouse.middle) {
                mjv_moveCamera(model, mjMOUSE_ZOOM, dx, dy, scene, cam);
            }
            break;
        }
        default:
            break;
        }
    }
}

void process_http_viewer_events(
    const std::vector<ViewerHttpEvent>& events,
    mjModel* model,
    mjData* data,
    mjvScene* scene,
    mjvOption* opt,
    mjvPerturb* perturb,
    mjvCamera* cam,
    RemoteViewerDragState& remote_drag,
    ViewerStats& stats,
    int fallback_body_id)
{
    for (const auto& event : events) {
        const double width = std::max(1.0, event.width);
        const double height = std::max(1.0, event.height);
        if (event.type == "down") {
            remote_drag.button = event.button;
            remote_drag.camera = false;
            remote_drag.perturb = false;

            const mjtNum aspect = static_cast<mjtNum>(width / height);
            const mjtNum relx = static_cast<mjtNum>(std::clamp(event.x / width, 0.0, 1.0));
            const mjtNum rely = static_cast<mjtNum>(std::clamp((height - event.y) / height, 0.0, 1.0));
            mjtNum selpnt[3] = {0, 0, 0};
            int selgeom = -1;
            int selflex = -1;
            int selskin = -1;
            int selbody = mjv_select(
                model,
                data,
                opt,
                aspect,
                relx,
                rely,
                scene,
                selpnt,
                &selgeom,
                &selflex,
                &selskin);

            const bool selected_from_view = selbody > 0;
            if (!selected_from_view && fallback_body_id > 0) {
                selbody = fallback_body_id;
                selflex = -1;
                selskin = -1;
                selpnt[0] = data->xpos[3 * selbody + 0];
                selpnt[1] = data->xpos[3 * selbody + 1];
                selpnt[2] = data->xpos[3 * selbody + 2];
            }

            if (selbody > 0) {
                perturb->select = selbody;
                perturb->flexselect = selflex;
                perturb->skinselect = selskin;
                mjtNum tmp[3];
                mju_sub3(tmp, selpnt, data->xpos + 3 * perturb->select);
                mju_mulMatTVec(perturb->localpos, data->xmat + 9 * perturb->select, tmp, 3, 3);
                mjv_initPerturb(model, data, scene, perturb);
                perturb->active = mjPERT_TRANSLATE;
                remote_drag.perturb = true;
                stats.last_perturb_body = selbody;
                stats.last_perturb_body_name = body_name(model, selbody);
                std::printf(
                    "[ViewerHTTP] perturb body=%d name=%s selected=%s\n",
                    selbody,
                    stats.last_perturb_body_name.c_str(),
                    selected_from_view ? "yes" : "fallback");
            } else {
                remote_drag.camera = true;
            }
            continue;
        }

        if (event.type == "move") {
            const double dx = event.dx / height;
            const double dy = event.dy / height;
            if (remote_drag.perturb && perturb->active) {
                const int action = remote_drag.button == 1 ? mjMOUSE_MOVE_V : mjMOUSE_MOVE_H;
                mjv_movePerturb(model, data, action, dx, -dy, scene, perturb);
            } else if (remote_drag.camera) {
                int action = mjMOUSE_ROTATE_H;
                if (remote_drag.button == 1) action = mjMOUSE_ZOOM;
                if (remote_drag.button == 2) action = mjMOUSE_MOVE_H;
                mjv_moveCamera(model, action, dx, dy, scene, cam);
            }
            continue;
        }

        if (event.type == "up" || event.type == "cancel") {
            if (remote_drag.perturb) {
                perturb->active = 0;
                remote_drag.perturb = false;
            }
            remote_drag.camera = false;
        }
    }
}

bool render_camera_frame(
    mjModel* model,
    mjData* data,
    mjvScene* scene,
    mjrContext* context,
    int camera_id,
    int width,
    int height,
    int jpeg_quality,
    std::vector<unsigned char>& jpg,
    std::vector<unsigned char>& png,
    std::vector<unsigned char>* rgb_raw = nullptr,
    std::vector<unsigned char>* rgba_raw = nullptr)
{
    if (camera_id < 0) return false;
    mjvCamera cam;
    mjvOption opt;
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    cam.type = mjCAMERA_FIXED;
    cam.fixedcamid = camera_id;

    mjrRect viewport{0, 0, width, height};
    mjr_setBuffer(mjFB_OFFSCREEN, context);
    if (context->currentBuffer != mjFB_OFFSCREEN) return false;
    mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, scene);
    mjr_render(viewport, scene, context);

    std::vector<unsigned char> rgb(static_cast<size_t>(width) * static_cast<size_t>(height) * 3);
    std::vector<float> depth(static_cast<size_t>(width) * static_cast<size_t>(height));
    mjr_readPixels(rgb.data(), depth.data(), viewport, context);
    mjr_setBuffer(mjFB_WINDOW, context);

    cv::Mat rgb_bottom(height, width, CV_8UC3, rgb.data());
    cv::Mat rgb_top;
    cv::flip(rgb_bottom, rgb_top, 0);
    if (rgb_raw) {
        rgb_raw->assign(rgb_top.datastart, rgb_top.dataend);
    }

    cv::Mat bgr;
    cv::cvtColor(rgb_top, bgr, cv::COLOR_RGB2BGR);
    std::vector<int> jpg_params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality};
    if (!cv::imencode(".jpg", bgr, jpg, jpg_params)) return false;

    cv::Mat rgba;
    cv::cvtColor(rgb_top, rgba, cv::COLOR_RGB2RGBA);
    if (rgba_raw) {
        rgba_raw->assign(rgba.datastart, rgba.dataend);
    }

    cv::Mat bgra;
    cv::cvtColor(rgb_top, bgra, cv::COLOR_RGB2BGRA);
    if (!cv::imencode(".png", bgra, png)) return false;
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    try {
        Args args = parse_args(argc, argv);
        const fs::path root = repo_root();
        load_mujoco_yaml(args, root);
        if (args.xml.empty()) args.xml = "assets/robots/magicbot_z1/scene.xml";

        const fs::path xml_path = resolve_path(root, args.xml);
        const fs::path config_path = resolve_path(root, args.config);
        auto loco_cfg = magicbot_loco::load_loco_config(config_path);
        magicbot_loco::ControllerCoreOptions core_options;
        core_options.safety.enabled = false;
        magicbot_loco::ControllerCore core(loco_cfg, core_options);
        core.warmup(20);

        magicbot_loco::JointArray stand_q = loco_cfg.default_motor();
        magicbot_loco::JointArray kp = loco_cfg.kps_motor();
        magicbot_loco::JointArray kd = loco_cfg.kds_motor();
        magicbot_loco::JointArray tau_limit = loco_cfg.tau_limit_motor();
        (void)load_initial_pose(root, args.initial_pose_yaml, stand_q, kp, kd, tau_limit);
        magicbot_loco::JointGains sim_gains;
        sim_gains.kp = kp;
        sim_gains.kd = kd;
        sim_gains.tau_limit = tau_limit;
        core.set_gains(sim_gains);
        core.set_default_target(stand_q);

        StateAndCmd external_state(magicbot_loco::kNumJoints);
        PolicyOutput external_output(magicbot_loco::kNumJoints);
        StateAndCmd skill_external_state(magicbot_loco::kNumJoints);
        PolicyOutput skill_external_output(magicbot_loco::kNumJoints);
        std::unique_ptr<BeyondMimicPolicy> beyond_policy;
        std::unique_ptr<BeyondMimicPolicy> track_mimic_policy;
        using BeyondAdapter =
            magicbot_loco::FsmExternalPolicyAdapter<BeyondMimicPolicy, StateAndCmd, PolicyOutput, FSMStateName>;
        std::unique_ptr<BeyondAdapter> beyond_adapter;
        std::unique_ptr<BeyondAdapter> track_mimic_adapter;
        bool dance_enabled = false;
        bool skill_enabled = false;
        if (!args.beyond_yaml.empty()) {
            const fs::path beyond_yaml_path = resolve_path(root, args.beyond_yaml);
            if (!fs::exists(beyond_yaml_path)) {
                throw std::runtime_error("BeyondMimic config not found: " + beyond_yaml_path.string());
            }
            beyond_policy = std::make_unique<BeyondMimicPolicy>(
                external_state,
                external_output,
                beyond_yaml_path.string(),
                static_cast<float>(loco_cfg.policy_dt));
            beyond_adapter = std::make_unique<BeyondAdapter>(
                magicbot_loco::ControlMode::Dance,
                magicbot_loco::kBeyondMimicPolicyKey,
                FSMStateName::SKILL_BEYOND_MIMIC,
                external_state,
                external_output,
                *beyond_policy,
                control_mode_for_fsm_state);
            core.register_external_policy(magicbot_loco::kBeyondMimicPolicyKey, *beyond_adapter, true);
            dance_enabled = true;
        }
        if (!args.track_mimic_yaml.empty()) {
            const fs::path track_yaml_path = resolve_path(root, args.track_mimic_yaml);
            if (!fs::exists(track_yaml_path)) {
                throw std::runtime_error("TrackMimic config not found: " + track_yaml_path.string());
            }
            track_mimic_policy = std::make_unique<BeyondMimicPolicy>(
                skill_external_state,
                skill_external_output,
                track_yaml_path.string(),
                static_cast<float>(loco_cfg.policy_dt),
                FSMStateName::SKILL_TRACK_MIMIC,
                "TrackMimic",
                false);
            track_mimic_adapter = std::make_unique<BeyondAdapter>(
                magicbot_loco::ControlMode::Skill,
                magicbot_loco::kTrackMimicPolicyKey,
                FSMStateName::SKILL_TRACK_MIMIC,
                skill_external_state,
                skill_external_output,
                *track_mimic_policy,
                control_mode_for_fsm_state);
            core.register_external_policy(magicbot_loco::kTrackMimicPolicyKey, *track_mimic_adapter, true);
            skill_enabled = true;
        }

        char error[1024] = {0};
        mjModel* model = mj_loadXML(xml_path.string().c_str(), nullptr, error, sizeof(error));
        if (!model) {
            std::fprintf(stderr, "mj_loadXML failed: %s\n", error);
            return 1;
        }
        mjData* data = mj_makeData(model);
        if (!data) {
            mj_deleteModel(model);
            throw std::runtime_error("mj_makeData failed");
        }
        model->opt.timestep = args.sim_dt;
        if (model->nu != magicbot_loco::kNumJoints) {
            std::fprintf(stderr, "Expected %d actuators, got %d\n", magicbot_loco::kNumJoints, model->nu);
            mj_deleteData(data);
            mj_deleteModel(model);
            return 1;
        }

        int camera_id = -1;
        if (args.camera_stream || args.camera_ros2) {
            camera_id = mj_name2id(model, mjOBJ_CAMERA, args.camera_name.c_str());
            if (camera_id < 0) {
                std::fprintf(stderr, "[Camera][WARN] camera not found: %s; camera outputs disabled\n",
                             args.camera_name.c_str());
                args.camera_stream = false;
                args.camera_ros2 = false;
            } else {
                model->vis.global.offwidth = args.camera_width;
                model->vis.global.offheight = args.camera_height;
            }
        }

        const auto qpos_idx = actuator_qpos_indices(model);
        const auto qvel_idx = actuator_qvel_indices(model);
        const int push_body_id = mj_name2id(model, mjOBJ_BODY, args.push_body.c_str());
        if ((has_vec3(args.push_force) || has_vec3(args.push_impulse)) && push_body_id < 0) {
            throw std::runtime_error("push body not found in MuJoCo model: " + args.push_body);
        }
        const int floor_geom_id = args.ground_correction
                                      ? mj_name2id(model, mjOBJ_GEOM, args.ground_floor_geom.c_str())
                                      : -1;
        const auto ground_contact_geom_ids =
            args.ground_correction ? resolve_contact_geom_ids(model, args.ground_body_keywords)
                                   : std::vector<int>{};
        if (args.ground_correction && floor_geom_id < 0) {
            std::fprintf(stderr, "[GroundCorrection][WARN] floor geom not found: %s\n",
                         args.ground_floor_geom.c_str());
        }
        reset_sim(model, data, qpos_idx, args.initial_base_height, stand_q);
        magicbot_loco::MujocoSimAdapterOptions sim_adapter_options;
        sim_adapter_options.qpos_idx = qpos_idx;
        sim_adapter_options.qvel_idx = qvel_idx;
        magicbot_loco::MujocoSimAdapter sim_adapter(model, data, sim_adapter_options);
        magicbot_loco::ControllerRuntime runtime(core, sim_adapter);
        core.reset_policy();

        XGlWindow win = create_window(args.width, args.height);
        mjvCamera cam;
        mjvOption opt;
        mjvPerturb perturb;
        mjvScene scene;
        mjvScene camera_scene;
        mjrContext context;
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultPerturb(&perturb);
        mjv_defaultScene(&scene);
        mjv_defaultScene(&camera_scene);
        mjr_defaultContext(&context);
        cam.type = mjCAMERA_FREE;
        cam.distance = 3.0;
        cam.azimuth = 135.0;
        cam.elevation = -18.0;
        cam.lookat[0] = 0.0;
        cam.lookat[1] = 0.0;
        cam.lookat[2] = 0.75;
        mjv_makeScene(model, &scene, 4000);
        if (args.camera_stream || args.camera_ros2) mjv_makeScene(model, &camera_scene, 2000);
        mjr_makeContext(model, &context, mjFONTSCALE_150);

        std::unique_ptr<CameraStreamServer> camera_server;
        if (args.camera_stream) {
            camera_server = std::make_unique<CameraStreamServer>(args.camera_host, args.camera_port);
            camera_server->start();
            std::printf("[CameraStream] %s/health\n", camera_server->url().c_str());
            std::printf("[CameraStream] %s/frame.jpg\n", camera_server->url().c_str());
            std::printf("[CameraStream] %s/frame.png\n", camera_server->url().c_str());
            std::printf("[CameraStream] %s/stream.mjpg\n", camera_server->url().c_str());
            std::printf("[CameraStream] POST %s/control?mode=loco&vx=0.2\n", camera_server->url().c_str());
        }
#ifdef ENABLE_ROS2_CAMERA
        std::unique_ptr<Ros2CameraPublisher> ros2_camera;
        if (args.camera_ros2) {
            ros2_camera = std::make_unique<Ros2CameraPublisher>(
                args.ros2_node_name,
                args.ros2_topic_rgb,
                args.ros2_topic_rgba,
                args.ros2_frame_id,
                args.ros2_qos_depth);
        }
#else
        if (args.camera_ros2) {
            std::fprintf(stderr, "[CameraROS2][WARN] ROS2 support was not enabled at build time\n");
            args.camera_ros2 = false;
        }
#endif

        bool running = true;
        bool loco_active = args.start_loco;
        bool passive_active = false;
        bool dance_active = false;
        bool skill_active = false;
        bool final_damping_active = false;
        bool paused = args.paused;
        bool follow = args.follow;
        bool reset_requested = false;
        std::array<float, 3> cmd{0.0f, 0.0f, 0.0f};
        std::unique_ptr<ViewerUdpCommandInput> udp_input;
        if (args.udp_control) {
            udp_input = std::make_unique<ViewerUdpCommandInput>(args, dance_enabled, skill_enabled);
        }
        MouseState mouse;
        RemoteViewerDragState remote_drag;
        ViewerStats stats;
        int sim_step = 0;
        int policy_step = 0;
        bool push_impulse_done = false;
        double root_x0 = data->qpos[0];
        double root_y0 = data->qpos[1];
        const int steps_per_frame = std::max(1, static_cast<int>(std::round(1.0 / (args.render_fps * args.sim_dt))));
        const double camera_period =
            (args.camera_stream || args.camera_ros2) ? 1.0 / static_cast<double>(args.camera_fps) : 1.0;
        const auto wall_start = Clock::now();
        auto last_print = wall_start;
        auto last_camera = wall_start - std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(camera_period));

        std::printf("=== MagicBot Z1 Native MuJoCo Loco Viewer ===\n");
        std::printf("XML    : %s\n", xml_path.string().c_str());
        std::printf("Config : %s\n", config_path.string().c_str());
        std::printf("ONNX   : %s\n", loco_cfg.policy_path.string().c_str());
        if (dance_enabled) {
            std::printf("Beyond : %s\n", resolve_path(root, args.beyond_yaml).string().c_str());
        }
        if (skill_enabled) {
            std::printf("Track  : %s (BeyondMimic trajectory mode)\n", resolve_path(root, args.track_mimic_yaml).string().c_str());
        }
        std::printf(
            "Keys   : L loco, M passive, N final damping, B beyond/dance, T track/skill, Space pause, R reset, "
            "F follow, X zero, W/S vx, Q/E vy, A/D wz, Esc close\n");
        if (udp_input) {
            std::printf(
                "UDP    : command input on %s:%d, timeout %.2fs "
                "(text: vx=0.3 vy=0 wz=0 mode=loco|passive|final_damping|track_mimic)\n",
                args.udp_bind.c_str(),
                args.udp_port,
                args.udp_timeout_s);
        }
        if (has_vec3(args.push_force) || has_vec3(args.push_impulse)) {
            std::printf(
                "Push   : body=%s force_norm=%.2f start=%.2fs duration=%.2fs impulse_norm=%.2f impulse_time=%.2fs\n",
                args.push_body.c_str(),
                vec3_norm(args.push_force),
                args.push_start_s,
                args.push_duration_s,
                vec3_norm(args.push_impulse),
                args.push_impulse_time_s);
        }
        if (!args.summary_json.empty()) {
            std::printf("Summary: %s\n", resolve_path(root, args.summary_json).string().c_str());
        }
        print_cmd(cmd, loco_active, passive_active, dance_active, skill_active, final_damping_active, paused);

        while (running) {
            const auto frame_start = Clock::now();
            process_events(
                win,
                model,
                data,
                &scene,
                &opt,
                &perturb,
                &cam,
                mouse,
                stats,
                running,
                loco_active,
                passive_active,
                dance_active,
                skill_active,
                final_damping_active,
                dance_enabled,
                skill_enabled,
                paused,
                follow,
                reset_requested,
                cmd);
            if (!running) break;
            if (camera_server) {
                if (camera_server->take_reset_request()) {
                    reset_requested = true;
                    ++stats.http_reset_requests;
                }
                const auto viewer_events = camera_server->take_viewer_events();
                if (!viewer_events.empty()) {
                    stats.http_viewer_events += static_cast<int>(viewer_events.size());
                    process_http_viewer_events(
                        viewer_events,
                        model,
                        data,
                        &scene,
                        &opt,
                        &perturb,
                        &cam,
                        remote_drag,
                        stats,
                        push_body_id);
                }
                const auto control_commands = camera_server->take_control_commands();
                if (!control_commands.empty()) {
                    stats.http_control_commands += static_cast<int>(control_commands.size());
                    for (const ViewerControlCommand& control : control_commands) {
                        apply_viewer_control_command(
                            control,
                            cmd,
                            loco_active,
                            passive_active,
                            dance_active,
                            skill_active,
                            final_damping_active,
                            dance_enabled,
                            skill_enabled,
                            paused,
                            running,
                            reset_requested);
                    }
                    print_cmd(cmd, loco_active, passive_active, dance_active, skill_active, final_damping_active, paused);
                }
            }
            if (udp_input &&
                udp_input->poll(
                    cmd,
                    loco_active,
                    passive_active,
                    dance_active,
                    skill_active,
                    final_damping_active,
                    paused,
                    running,
                    reset_requested)) {
                print_cmd(cmd, loco_active, passive_active, dance_active, skill_active, final_damping_active, paused);
            }
            if (!running) break;

            if (reset_requested) {
                reset_sim(model, data, qpos_idx, args.initial_base_height, stand_q);
                core.set_default_target(stand_q);
                core.reset_policy();
                sim_step = 0;
                policy_step = 0;
                push_impulse_done = false;
                root_x0 = data->qpos[0];
                root_y0 = data->qpos[1];
                stats.min_base_height = std::numeric_limits<double>::infinity();
                stats.max_root_xy_drift = 0.0;
                stats.max_gravity_xy = 0.0;
                reset_requested = false;
            }

            if (!paused) {
                for (int i = 0; i < steps_per_frame; ++i) {
                    magicbot_loco::RuntimeTickInput tick_input;
                    tick_input.command.velocity = cmd;
                    tick_input.mode_request =
                        viewer_mode_request(loco_active, passive_active, dance_active, skill_active, final_damping_active);
                    tick_input.control_dt_s = static_cast<float>(model->opt.timestep);
                    tick_input.publish_target = true;
                    const auto tick = runtime.tick(tick_input);
                    sync_viewer_mode_flags(
                        tick.core.telemetry.mode,
                        loco_active,
                        passive_active,
                        dance_active,
                        skill_active,
                        final_damping_active);
                    policy_step = tick.core.telemetry.policy_steps;
                    const auto gravity = tick.core.telemetry.projected_gravity;
                    stats.max_gravity_xy = std::max(
                        stats.max_gravity_xy,
                        std::sqrt(static_cast<double>(gravity[0]) * gravity[0] +
                                  static_cast<double>(gravity[1]) * gravity[1]));
                    if (model->nbody > 0 && data->xfrc_applied != nullptr) {
                        mju_zero(data->xfrc_applied, 6 * model->nbody);
                        if (perturb.active) {
                            mjv_applyPerturbForce(model, data, &perturb);
                            ++stats.mouse_perturb_steps;
                            stats.last_perturb_body = perturb.select;
                            stats.last_perturb_body_name = body_name(model, perturb.select);
                        }
                        const PushStepResult push =
                            apply_push_disturbance(args, model, data, push_body_id, push_impulse_done);
                        if (push.force_active) ++stats.push_force_steps;
                        if (push.impulse_applied) stats.push_impulse_applied = true;
                    }
                    mj_step(model, data);
                    if (args.ground_correction) {
                        (void)correct_ground_penetration(
                            model,
                            data,
                            floor_geom_id,
                            ground_contact_geom_ids,
                            args.ground_max_penetration);
                    }
                    stats.min_base_height = std::min(stats.min_base_height, static_cast<double>(data->qpos[2]));
                    stats.max_root_xy_drift = std::max(
                        stats.max_root_xy_drift,
                        std::hypot(data->qpos[0] - root_x0, data->qpos[1] - root_y0));
                    ++sim_step;
                }
            } else {
                mj_forward(model, data);
            }

            if (follow) {
                cam.lookat[0] = data->qpos[0];
                cam.lookat[1] = data->qpos[1];
                cam.lookat[2] = std::max<mjtNum>(0.45, data->qpos[2]);
            }

            if (camera_server) {
                camera_server->update_status(
                    viewer_mode_label(loco_active, passive_active, dance_active, skill_active, final_damping_active),
                    paused,
                    cmd,
                    sim_step,
                    policy_step,
                    data->time,
                    data->qpos[0],
                    data->qpos[1],
                    data->qpos[2],
                    stats);
            }

            const auto now_before_render = Clock::now();
            if ((args.camera_stream || args.camera_ros2) &&
                std::chrono::duration<double>(now_before_render - last_camera).count() >= camera_period) {
                std::vector<unsigned char> jpg;
                std::vector<unsigned char> png;
                std::vector<unsigned char> rgb;
                std::vector<unsigned char> rgba;
                if (render_camera_frame(
                        model,
                        data,
                        &camera_scene,
                        &context,
                        camera_id,
                        args.camera_width,
                        args.camera_height,
                        args.jpeg_quality,
                        jpg,
                        png,
                        args.camera_ros2 ? &rgb : nullptr,
                        args.camera_ros2 ? &rgba : nullptr)) {
                    const double ts = std::chrono::duration<double>(now_before_render.time_since_epoch()).count();
                    if (camera_server) camera_server->update(std::move(jpg), std::move(png), ts);
#ifdef ENABLE_ROS2_CAMERA
                    if (ros2_camera) {
                        ros2_camera->publish(rgb, rgba, args.camera_width, args.camera_height);
                    }
#endif
                }
                last_camera = now_before_render;
            }

            mjrRect viewport{0, 0, win.width, win.height};
            mjr_setBuffer(mjFB_WINDOW, &context);
            mjv_updateScene(model, data, &opt, &perturb, &cam, mjCAT_ALL, &scene);
            mjr_render(viewport, &scene, &context);

            char left[512];
            char right[512];
            const double wall_s = std::chrono::duration<double>(Clock::now() - wall_start).count();
            std::snprintf(
                left,
                sizeof(left),
                "mode: %s\npause: %s\nsim: %.2fs\nbase: %.2f %.2f %.3f\npolicy steps: %d",
                viewer_mode_label(loco_active, passive_active, dance_active, skill_active, final_damping_active),
                paused ? "yes" : "no",
                data->time,
                data->qpos[0],
                data->qpos[1],
                data->qpos[2],
                policy_step);
            std::snprintf(
                right,
                sizeof(right),
                "cmd vx %.2f\ncmd vy %.2f\ncmd wz %.2f\npush steps %d impulse %s\nperturb %s\nL loco | M passive | N final\nB dance | T skill | R reset | X zero",
                cmd[0],
                cmd[1],
                cmd[2],
                stats.push_force_steps,
                stats.push_impulse_applied ? "yes" : "no",
                perturb.active ? body_name(model, perturb.select).c_str() : "off");
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, left, "", &context);
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, right, "", &context);
            glXSwapBuffers(win.display, win.window);

            const auto now = Clock::now();
            if (std::chrono::duration<double>(now - last_print).count() >= 1.0) {
                last_print = now;
                std::printf(
                    "[Viewer] wall=%.1f sim=%.2f mode=%s cmd=[%.2f %.2f %.2f] base=[%.2f %.2f %.3f]\n",
                    wall_s,
                    data->time,
                    viewer_mode_label(loco_active, passive_active, dance_active, skill_active, final_damping_active),
                    cmd[0],
                    cmd[1],
                    cmd[2],
                    data->qpos[0],
                    data->qpos[1],
                    data->qpos[2]);
                std::fflush(stdout);
            }
            if (args.duration > 0.0 && wall_s >= args.duration) running = false;

            if (args.realtime) {
                const double frame_dt = 1.0 / static_cast<double>(args.render_fps);
                const double elapsed = std::chrono::duration<double>(Clock::now() - frame_start).count();
                if (elapsed < frame_dt) {
                    std::this_thread::sleep_for(std::chrono::duration<double>(frame_dt - elapsed));
                }
            }
        }

        if (!args.summary_json.empty()) {
            const double wall_s = std::chrono::duration<double>(Clock::now() - wall_start).count();
            const std::string json = viewer_summary_json(
                args,
                model,
                data,
                stats,
                sim_step,
                policy_step,
                loco_active,
                passive_active,
                dance_active,
                skill_active,
                final_damping_active,
                paused,
                wall_s,
                push_body_id);
            const fs::path summary_path = resolve_path(root, args.summary_json);
            std::ofstream out(summary_path);
            out << json << "\n";
            std::printf("[Summary] %s\n", summary_path.string().c_str());
        }

        if (camera_server) camera_server->stop();
        mjr_freeContext(&context);
        if (args.camera_stream || args.camera_ros2) mjv_freeScene(&camera_scene);
        mjv_freeScene(&scene);
        destroy_window(win);
        mj_deleteData(data);
        mj_deleteModel(model);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[mujoco_loco_viewer][ERROR] %s\n", e.what());
        return 1;
    }
}
