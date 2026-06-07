# Runtime Split

This deploy tree is being split into five independent layers:

1. Rendering: visualization only. MuJoCo viewer, ghost display, cameras.
2. Communication: transport only. DDS shadow topics or MagicBot SDK state/command APIs.
3. Control: deterministic command math. PD torque, finite checks, action/gain safety.
4. Inference: policy/FSM output. ONNX/FSM state machine, no transport or rendering.
5. Backend: world source/sink. Simulation uses MuJoCo state and actuator writes; real uses MagicBot SDK state and low-level command writes.

Simulation path:

```text
MuJoCo state -> inference/FSM -> safety/control -> MuJoCo actuator
             -> DDS shadow communication -> C++ compare
             -> rendering
```

Real path:

```text
MagicBot SDK state -> inference/FSM or SDK ONNX wrapper -> safety/control -> MagicBot SDK command
```

Rules:

- Rendering must be optional and never required for real deployment.
- Communication modules must not run policy logic.
- Inference modules must not publish robot commands.
- Control modules must not know whether the state came from MuJoCo or the robot.
- Real deployment starts with non-command modes: dry-run, connect-check, read-state.

