#include "magicbot_loco_core.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <netinet/in.h>
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

constexpr int kHeadMotorIndex = 13;

struct Args {
    std::string config = "policies/loco_mode/config/LocoMode_lowKp.yaml";
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
    int last_x{0};
    int last_y{0};
};

struct CameraStreamState {
    std::mutex mutex;
    std::vector<unsigned char> latest_jpg;
    std::vector<unsigned char> latest_png;
    double timestamp{0.0};
    uint64_t seq{0};
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
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\n\r\n";
        send_all(fd, header.data(), header.size());
        if (!body.empty()) send_all(fd, reinterpret_cast<const char*>(body.data()), body.size());
    }

    static std::string request_path(const std::string& request)
    {
        const size_t first_space = request.find(' ');
        if (first_space == std::string::npos) return "/";
        const size_t second_space = request.find(' ', first_space + 1);
        if (second_space == std::string::npos) return "/";
        return request.substr(first_space + 1, second_space - first_space - 1);
    }

    static void handle_client(int fd, std::shared_ptr<CameraStreamState> state)
    {
        char buffer[4096] = {0};
        ssize_t n = recv(fd, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0) {
            close(fd);
            return;
        }
        const std::string path = request_path(std::string(buffer, static_cast<size_t>(n)));
        if (path == "/health") {
            std::lock_guard<std::mutex> lock(state->mutex);
            std::string body = "{\"ok\":true,\"seq\":" + std::to_string(state->seq) +
                               ",\"timestamp\":" + std::to_string(state->timestamp) + "}\n";
            send_text(fd, 200, "OK", body, "application/json");
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
        "Usage: %s [--config PATH] [--mujoco-yaml PATH] [--xml PATH]\n"
        "          [--initial-pose-yaml PATH] [--loco] [--paused|--unpaused]\n"
        "          [--duration SEC] [--width N] [--height N]\n"
        "          [--camera-stream] [--camera-port N] [--camera-name NAME]\n"
        "          [--camera-ros2] [--ros2-topic-rgb TOPIC] [--ros2-topic-rgba TOPIC]\n"
        "\n"
        "Keys: L toggle loco, Space pause, R reset, F follow, X zero command,\n"
        "      W/S vx, Q/E vy, A/D wz, Esc close. Mouse drags move camera.\n",
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
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--config") {
            args.config = need_value(i, argc, argv);
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
    args.camera_port = std::max(1, args.camera_port);
    args.camera_width = std::max(64, args.camera_width);
    args.camera_height = std::max(64, args.camera_height);
    args.camera_fps = std::max(1, args.camera_fps);
    args.jpeg_quality = std::clamp(args.jpeg_quality, 1, 100);
    args.ros2_qos_depth = std::max(1, args.ros2_qos_depth);
    if (args.sim_dt <= 0.0) args.sim_dt = 0.002;
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

magicbot_loco::JointArray read_q(const mjData* data, const std::vector<int>& qpos_idx)
{
    magicbot_loco::JointArray out{};
    for (int i = 0; i < magicbot_loco::kNumJoints; ++i) out[i] = static_cast<float>(data->qpos[7 + qpos_idx[i]]);
    return out;
}

magicbot_loco::JointArray read_dq(const mjData* data, const std::vector<int>& qvel_idx)
{
    magicbot_loco::JointArray out{};
    for (int i = 0; i < magicbot_loco::kNumJoints; ++i) out[i] = static_cast<float>(data->qvel[6 + qvel_idx[i]]);
    return out;
}

std::array<float, 4> read_quat(const mjData* data)
{
    return {
        static_cast<float>(data->qpos[3]),
        static_cast<float>(data->qpos[4]),
        static_cast<float>(data->qpos[5]),
        static_cast<float>(data->qpos[6]),
    };
}

std::array<float, 3> read_ang_vel(const mjModel* model, const mjData* data, int body_id)
{
    (void)model;
    (void)body_id;
    return {
        static_cast<float>(data->qvel[3]),
        static_cast<float>(data->qvel[4]),
        static_cast<float>(data->qvel[5]),
    };
}

void apply_pd(
    mjModel* model,
    mjData* data,
    const std::vector<int>& qpos_idx,
    const std::vector<int>& qvel_idx,
    const magicbot_loco::JointArray& target,
    const magicbot_loco::JointArray& kp,
    const magicbot_loco::JointArray& kd,
    const magicbot_loco::JointArray& tau_limit)
{
    for (int i = 0; i < model->nu && i < magicbot_loco::kNumJoints; ++i) {
        const double q = data->qpos[7 + qpos_idx[i]];
        const double dq = data->qvel[6 + qvel_idx[i]];
        const double target_q = (i == kHeadMotorIndex) ? 0.0 : static_cast<double>(target[static_cast<size_t>(i)]);
        const double lim = std::max(0.0f, tau_limit[static_cast<size_t>(i)]);
        double tau = (target_q - q) * static_cast<double>(kp[static_cast<size_t>(i)]) -
                     dq * static_cast<double>(kd[static_cast<size_t>(i)]);
        tau = std::clamp(tau, -lim, lim);
        if (model->actuator_ctrllimited && model->actuator_ctrllimited[i]) {
            const double lo = model->actuator_ctrlrange[2 * i + 0];
            const double hi = model->actuator_ctrlrange[2 * i + 1];
            tau = std::clamp(tau, lo, hi);
        }
        data->ctrl[i] = std::isfinite(tau) ? tau : 0.0;
    }
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

void print_cmd(const std::array<float, 3>& cmd, bool loco, bool paused)
{
    std::printf(
        "[Viewer] mode=%s paused=%s cmd(vx,vy,wz)=[%.2f %.2f %.2f]\n",
        loco ? "LOCO" : "STAND",
        paused ? "yes" : "no",
        cmd[0],
        cmd[1],
        cmd[2]);
}

void handle_key(
    KeySym sym,
    bool& running,
    bool& loco_active,
    bool& paused,
    bool& follow,
    bool& reset_requested,
    std::array<float, 3>& cmd,
    magicbot_loco::OnnxLocoPolicy& policy)
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
        policy.reset();
        break;
    case XK_r:
    case XK_R:
        reset_requested = true;
        policy.reset();
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
    print_cmd(cmd, loco_active, paused);
}

void process_events(
    XGlWindow& win,
    mjModel* model,
    mjvScene* scene,
    mjvCamera* cam,
    MouseState& mouse,
    bool& running,
    bool& loco_active,
    bool& paused,
    bool& follow,
    bool& reset_requested,
    std::array<float, 3>& cmd,
    magicbot_loco::OnnxLocoPolicy& policy)
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
            handle_key(sym, running, loco_active, paused, follow, reset_requested, cmd, policy);
            break;
        }
        case ButtonPress:
            mouse.last_x = event.xbutton.x;
            mouse.last_y = event.xbutton.y;
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
            if (event.xbutton.button == Button1) mouse.left = false;
            if (event.xbutton.button == Button2) mouse.middle = false;
            if (event.xbutton.button == Button3) mouse.right = false;
            break;
        case MotionNotify: {
            const double dx = static_cast<double>(event.xmotion.x - mouse.last_x) / std::max(1, win.height);
            const double dy = static_cast<double>(event.xmotion.y - mouse.last_y) / std::max(1, win.height);
            mouse.last_x = event.xmotion.x;
            mouse.last_y = event.xmotion.y;
            if (mouse.left) {
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
        magicbot_loco::OnnxLocoPolicy policy(loco_cfg);
        policy.warmup(20);

        magicbot_loco::JointArray stand_q = loco_cfg.default_motor();
        magicbot_loco::JointArray kp = loco_cfg.kps_motor();
        magicbot_loco::JointArray kd = loco_cfg.kds_motor();
        magicbot_loco::JointArray tau_limit = loco_cfg.tau_limit_motor();
        (void)load_initial_pose(root, args.initial_pose_yaml, stand_q, kp, kd, tau_limit);

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
        const int root_body_id = mj_name2id(model, mjOBJ_BODY, "pelvis");
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

        XGlWindow win = create_window(args.width, args.height);
        mjvCamera cam;
        mjvOption opt;
        mjvScene scene;
        mjvScene camera_scene;
        mjrContext context;
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
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
        bool paused = args.paused;
        bool follow = args.follow;
        bool reset_requested = false;
        std::array<float, 3> cmd{0.0f, 0.0f, 0.0f};
        MouseState mouse;
        magicbot_loco::JointArray target = stand_q;
        magicbot_loco::JointArray previous_raw_target = stand_q;
        int sim_step = 0;
        int policy_step = 0;
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
        std::printf("Keys   : L loco, Space pause, R reset, F follow, X zero, W/S vx, Q/E vy, A/D wz, Esc close\n");
        print_cmd(cmd, loco_active, paused);

        while (running) {
            const auto frame_start = Clock::now();
            process_events(
                win,
                model,
                &scene,
                &cam,
                mouse,
                running,
                loco_active,
                paused,
                follow,
                reset_requested,
                cmd,
                policy);
            if (!running) break;

            if (reset_requested) {
                reset_sim(model, data, qpos_idx, args.initial_base_height, stand_q);
                target = stand_q;
                previous_raw_target = stand_q;
                policy.reset();
                sim_step = 0;
                policy_step = 0;
                reset_requested = false;
            }

            if (!paused) {
                for (int i = 0; i < steps_per_frame; ++i) {
                    const bool control_tick = (sim_step % args.control_decimation) == 0;
                    if (control_tick) {
                        const auto q = read_q(data, qpos_idx);
                        const auto dq = read_dq(data, qvel_idx);
                        if (loco_active) {
                            const auto quat = read_quat(data);
                            const auto gravity = magicbot_loco::gravity_orientation(quat);
                            const auto ang_vel = read_ang_vel(model, data, root_body_id);
                            auto infer = policy.infer(q, dq, ang_vel, gravity, cmd);
                            magicbot_loco::JointArray raw = infer.target_motor;
                            target = magicbot_loco::torque_limited_target(
                                raw, q, dq, kp, kd, tau_limit, loco_cfg.tau_limit_scale);
                            previous_raw_target = raw;
                            ++policy_step;
                        } else {
                            target = stand_q;
                        }
                    }
                    apply_pd(model, data, qpos_idx, qvel_idx, target, kp, kd, tau_limit);
                    mj_step(model, data);
                    if (args.ground_correction) {
                        (void)correct_ground_penetration(
                            model,
                            data,
                            floor_geom_id,
                            ground_contact_geom_ids,
                            args.ground_max_penetration);
                    }
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
            mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scene);
            mjr_render(viewport, &scene, &context);

            char left[512];
            char right[512];
            const double wall_s = std::chrono::duration<double>(Clock::now() - wall_start).count();
            std::snprintf(
                left,
                sizeof(left),
                "mode: %s\npause: %s\nsim: %.2fs\nbase z: %.3f\npolicy steps: %d",
                loco_active ? "LOCO" : "STAND",
                paused ? "yes" : "no",
                data->time,
                data->qpos[2],
                policy_step);
            std::snprintf(
                right,
                sizeof(right),
                "cmd vx %.2f\ncmd vy %.2f\ncmd wz %.2f\nL loco | Space pause\nR reset | F follow | X zero",
                cmd[0],
                cmd[1],
                cmd[2]);
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, left, "", &context);
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, right, "", &context);
            glXSwapBuffers(win.display, win.window);

            const auto now = Clock::now();
            if (std::chrono::duration<double>(now - last_print).count() >= 1.0) {
                last_print = now;
                std::printf(
                    "[Viewer] wall=%.1f sim=%.2f mode=%s cmd=[%.2f %.2f %.2f] z=%.3f\n",
                    wall_s,
                    data->time,
                    loco_active ? "LOCO" : "STAND",
                    cmd[0],
                    cmd[1],
                    cmd[2],
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
