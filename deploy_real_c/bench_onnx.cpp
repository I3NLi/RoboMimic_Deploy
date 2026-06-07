#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <numeric>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace {

struct Args {
    std::string model;
    int iters = 5000;
    int warmup = 200;
    int threads = 1;
    int obs_dim = 0;
};

void usage(const char* argv0)
{
    std::printf(
        "Usage: %s --model PATH [--iters N] [--warmup N] [--threads N] [--obs-dim N]\n",
        argv0);
}

Args parse_args(int argc, char** argv)
{
    Args args;
    for (int i = 1; i < argc; i++) {
        auto need_value = [&](const char* opt) {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s requires a value\n", opt);
                std::exit(2);
            }
            return argv[++i];
        };
        if (std::strcmp(argv[i], "--model") == 0) {
            args.model = need_value(argv[i]);
        } else if (std::strcmp(argv[i], "--iters") == 0) {
            args.iters = std::atoi(need_value(argv[i]));
        } else if (std::strcmp(argv[i], "--warmup") == 0) {
            args.warmup = std::atoi(need_value(argv[i]));
        } else if (std::strcmp(argv[i], "--threads") == 0) {
            args.threads = std::atoi(need_value(argv[i]));
        } else if (std::strcmp(argv[i], "--obs-dim") == 0) {
            args.obs_dim = std::atoi(need_value(argv[i]));
        } else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            std::exit(0);
        } else {
            std::fprintf(stderr, "Unknown arg: %s\n", argv[i]);
            usage(argv[0]);
            std::exit(2);
        }
    }
    if (args.model.empty()) {
        usage(argv[0]);
        std::exit(2);
    }
    args.iters = std::max(1, args.iters);
    args.warmup = std::max(0, args.warmup);
    args.threads = std::max(1, args.threads);
    return args;
}

std::vector<int64_t> concrete_shape(const std::vector<int64_t>& raw, int fallback_dim)
{
    std::vector<int64_t> shape;
    shape.reserve(raw.size());
    for (size_t i = 0; i < raw.size(); i++) {
        if (raw[i] > 0) {
            shape.push_back(raw[i]);
        } else if (i == 0) {
            shape.push_back(1);
        } else if (fallback_dim > 0) {
            shape.push_back(fallback_dim);
        } else {
            shape.push_back(1);
        }
    }
    return shape;
}

size_t numel(const std::vector<int64_t>& shape)
{
    size_t n = 1;
    for (auto d : shape) n *= static_cast<size_t>(std::max<int64_t>(1, d));
    return n;
}

double percentile(std::vector<double> samples, double q)
{
    if (samples.empty()) return 0.0;
    std::sort(samples.begin(), samples.end());
    const size_t idx = std::min(
        samples.size() - 1,
        static_cast<size_t>((samples.size() - 1) * q + 0.5));
    return samples[idx];
}

}  // namespace

int main(int argc, char** argv)
{
    Args args = parse_args(argc, argv);

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "bench_onnx");
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(args.threads);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    Ort::Session session(env, args.model.c_str(), opts);
    Ort::AllocatorWithDefaultOptions alloc;
    Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

    const size_t n_inputs = session.GetInputCount();
    const size_t n_outputs = session.GetOutputCount();
    std::vector<std::string> input_names_owned;
    std::vector<std::string> output_names_owned;
    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<std::vector<float>> input_buffers;

    for (size_t i = 0; i < n_inputs; i++) {
        auto name = session.GetInputNameAllocated(i, alloc);
        input_names_owned.emplace_back(name ? name.get() : "");
        if (args.obs_dim > 0) {
            input_shapes.push_back(i == 0
                ? std::vector<int64_t>{1, args.obs_dim}
                : std::vector<int64_t>{1, 1});
        } else {
            auto info = session.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo();
            int fallback = (i == 0) ? args.obs_dim : 1;
            input_shapes.push_back(concrete_shape(info.GetShape(), fallback));
        }
        input_buffers.emplace_back(numel(input_shapes.back()), 0.0f);
    }
    for (size_t i = 0; i < n_outputs; i++) {
        auto name = session.GetOutputNameAllocated(i, alloc);
        output_names_owned.emplace_back(name ? name.get() : "");
    }
    for (auto& s : input_names_owned) input_names.push_back(s.c_str());
    for (auto& s : output_names_owned) output_names.push_back(s.c_str());

    auto run_once = [&](int step) {
        if (input_buffers.size() >= 2 && !input_buffers[1].empty()) {
            input_buffers[1][0] = static_cast<float>(step);
        }
        std::vector<Ort::Value> inputs;
        inputs.reserve(input_buffers.size());
        for (size_t i = 0; i < input_buffers.size(); i++) {
            inputs.push_back(Ort::Value::CreateTensor<float>(
                mem,
                input_buffers[i].data(),
                input_buffers[i].size(),
                input_shapes[i].data(),
                input_shapes[i].size()));
        }
        return session.Run(
            Ort::RunOptions{nullptr},
            input_names.data(),
            inputs.data(),
            inputs.size(),
            output_names.data(),
            output_names.size());
    };

    for (int i = 0; i < args.warmup; i++) {
        auto outputs = run_once(i);
        (void)outputs;
    }

    std::vector<double> samples;
    samples.reserve(static_cast<size_t>(args.iters));
    auto total0 = std::chrono::steady_clock::now();
    for (int i = 0; i < args.iters; i++) {
        auto t0 = std::chrono::steady_clock::now();
        auto outputs = run_once(i);
        (void)outputs;
        auto t1 = std::chrono::steady_clock::now();
        samples.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    auto total1 = std::chrono::steady_clock::now();

    double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    auto minmax = std::minmax_element(samples.begin(), samples.end());
    std::vector<double> sorted = samples;
    std::sort(sorted.begin(), sorted.end());
    double median = sorted[sorted.size() / 2];
    double total_ms = std::chrono::duration<double, std::milli>(total1 - total0).count();

    std::printf("runtime=cpp\n");
    std::printf("model=%s\n", args.model.c_str());
    std::printf("inputs=");
    for (size_t i = 0; i < input_names_owned.size(); i++) {
        std::printf("%s%s[", i ? "," : "", input_names_owned[i].c_str());
        for (size_t j = 0; j < input_shapes[i].size(); j++) {
            std::printf("%s%lld", j ? "x" : "", static_cast<long long>(input_shapes[i][j]));
        }
        std::printf("]");
    }
    std::printf("\n");
    std::printf("outputs=%zu\n", n_outputs);
    std::printf("threads=%d warmup=%d iters=%d\n", args.threads, args.warmup, args.iters);
    std::printf("mean_ms=%.6f\n", sum / static_cast<double>(samples.size()));
    std::printf("median_ms=%.6f\n", median);
    std::printf("p95_ms=%.6f\n", percentile(samples, 0.95));
    std::printf("p99_ms=%.6f\n", percentile(samples, 0.99));
    std::printf("min_ms=%.6f\n", *minmax.first);
    std::printf("max_ms=%.6f\n", *minmax.second);
    std::printf("throughput_hz=%.2f\n", args.iters / std::max(total_ms / 1000.0, 1e-9));
    return 0;
}
