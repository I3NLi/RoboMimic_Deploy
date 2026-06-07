# Inference Benchmark - 2026-06-07

Host: local x86 workstation, Intel Core i9-14900KF.

Scope: pure ONNXRuntime inference only. This excludes MuJoCo rendering, DDS communication,
FSM transition logic, safety filters, and real robot SDK communication.

Build:

```bash
cmake -S deploy_real_c -B deploy_real_c/build_z1
cmake --build deploy_real_c/build_z1 --target bench_onnx
```

Commands:

```bash
/home/hiyio/anaconda3/envs/robomimic/bin/python tools/bench_onnx.py \
  --model policy/loco_mode/model/z1_flat_8192_rootheight_model_2700.onnx \
  --iters 10000 --warmup 500 --threads 0 --obs-dim 82

/home/hiyio/anaconda3/envs/robomimic/bin/python tools/bench_onnx.py \
  --model policy/beyond_mimic/model/policy.onnx \
  --iters 5000 --warmup 500 --threads 0 --obs-dim 124

deploy_real_c/build_z1/bench_onnx \
  --model policy/loco_mode/model/z1_flat_8192_rootheight_model_2700.onnx \
  --obs-dim 82 --iters 10000 --warmup 500 --threads 1

deploy_real_c/build_z1/bench_onnx \
  --model policy/beyond_mimic/model/policy.onnx \
  --obs-dim 124 --iters 5000 --warmup 500 --threads 1
```

Results:

| Model | Runtime | Threads | Mean ms | Median ms | P95 ms | P99 ms | Throughput Hz |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Loco `82 -> 24` | Python ORT | default | 0.014109 | 0.011464 | 0.018880 | 0.065649 | 69939 |
| Loco `82 -> 24` | Python ORT | 1 | 0.013423 | 0.013251 | 0.014252 | 0.018426 | 73727 |
| Loco `82 -> 24` | C++ ORT | 1 | 0.011975 | 0.011060 | 0.016317 | 0.017703 | 82581 |
| BeyondMimic `124 + step -> 7 outputs` | Python ORT | default | 0.019246 | 0.017958 | 0.025254 | 0.030651 | 50831 |
| BeyondMimic `124 + step -> 7 outputs` | Python ORT | 1 | 0.052344 | 0.041255 | 0.096215 | 0.223627 | 18717 |
| BeyondMimic `124 + step -> 7 outputs` | C++ ORT | 1 | 0.017351 | 0.015836 | 0.023768 | 0.044517 | 55487 |

Conclusion:

- Pure ONNX inference is far below the 20 ms budget for a 50 Hz control loop.
- On this x86 host, C++ ORT is slightly faster than Python ORT default for both models.
- The meaningful runtime difference will more likely come from Python-side FSM/observation code,
  MuJoCo/rendering, DDS, or SDK communication rather than the ONNX inference call itself.
- Jetson/RK results must be measured separately after `192.168.0.103` is reachable.

