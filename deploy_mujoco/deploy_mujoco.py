import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from common.path_config import PROJECT_ROOT

import time
import mujoco.viewer
import mujoco
import numpy as np
import yaml
import os
from common.ctrlcomp import *
from FSM.FSM import *
from common.utils import get_gravity_orientation
from common.joystick import JoyStick, JoystickButton
from common.safety import load_safety_config, SafetyFilter, HoldToConfirm

try:
    from deploy_mujoco.head_camera_stream import HeadCameraStreamServer
    _head_camera_stream_import_error = None
except Exception as e:
    try:
        # Fallback when running as a script: python deploy_mujoco/deploy_mujoco.py
        from head_camera_stream import HeadCameraStreamServer
        _head_camera_stream_import_error = None
    except Exception as e_local:
        HeadCameraStreamServer = None
        _head_camera_stream_import_error = e_local

try:
    from deploy_mujoco.offscreen_renderer import OffscreenCameraRenderer
    _offscreen_renderer_import_error = None
except Exception as e:
    try:
        # Fallback when running as a script: python deploy_mujoco/deploy_mujoco.py
        from offscreen_renderer import OffscreenCameraRenderer
        _offscreen_renderer_import_error = None
    except Exception as e_local:
        OffscreenCameraRenderer = None
        _offscreen_renderer_import_error = e_local


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd


def sanitize_ctrl(ctrl, model, fallback=None):
    """Ensure finite actuator controls and clamp to actuator ctrlrange if available."""
    out = np.asarray(ctrl, dtype=np.float32).reshape(-1)
    if fallback is None:
        fallback = np.zeros_like(out)
    fallback = np.asarray(fallback, dtype=np.float32).reshape(-1)
    if fallback.size != out.size:
        fallback = np.zeros_like(out)

    finite_mask = np.isfinite(out)
    if not np.all(finite_mask):
        out = np.where(finite_mask, out, fallback)
    out = np.nan_to_num(out, nan=0.0, posinf=0.0, neginf=0.0)

    ctrl_range = getattr(model, "actuator_ctrlrange", None)
    if ctrl_range is not None and ctrl_range.shape[0] == out.size:
        lo = ctrl_range[:, 0]
        hi = ctrl_range[:, 1]
        # Some models may leave ctrlrange unset (lo==hi==0), fallback in that case.
        if np.any(hi > lo):
            return np.clip(out, lo, hi).astype(np.float32)

    return np.clip(out, -300.0, 300.0).astype(np.float32)


def randomize_cube_obstacles(model, data, cfg):
    """Randomize cube obstacle poses/colors by body name prefix.

    Supports both dynamic cubes (freejoint) and static cubes.
    """
    if not bool(cfg.get("enable", False)):
        return 0

    prefix = str(cfg.get("body_prefix", "cube_rand_"))
    x_min = float(cfg.get("x_min", 0.45))
    x_max = float(cfg.get("x_max", 1.55))
    y_min = float(cfg.get("y_min", -0.85))
    y_max = float(cfg.get("y_max", 0.85))
    z_offset = float(cfg.get("z_offset", 0.003))
    seed = int(cfg.get("seed", -1))
    rng = np.random.default_rng(None if seed < 0 else seed)

    randomized = 0
    for body_id in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if not body_name or not body_name.startswith(prefix):
            continue

        jnt_num = int(model.body_jntnum[body_id])
        has_free_joint = False
        qpos_adr = -1
        dof_adr = -1
        if jnt_num >= 1:
            jnt_id = int(model.body_jntadr[body_id])
            has_free_joint = model.jnt_type[jnt_id] == mujoco.mjtJoint.mjJNT_FREE
            if has_free_joint:
                qpos_adr = int(model.jnt_qposadr[jnt_id])
                dof_adr = int(model.jnt_dofadr[jnt_id])

        geom_adr = int(model.body_geomadr[body_id])
        geom_num = int(model.body_geomnum[body_id])
        half_height = 0.03
        for geom_id in range(geom_adr, geom_adr + geom_num):
            if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_BOX:
                half_height = max(half_height, float(model.geom_size[geom_id, 2]))

        # Keep cubes on/above floor, spread in front of robot.
        x = float(rng.uniform(x_min, x_max))
        y = float(rng.uniform(y_min, y_max))
        z = half_height + z_offset
        if has_free_joint:
            data.qpos[qpos_adr : qpos_adr + 7] = np.array([x, y, z, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)
            data.qvel[dof_adr : dof_adr + 6] = 0.0
        else:
            # Static obstacle body: randomize model-space transform directly.
            model.body_pos[body_id, 0:3] = np.array([x, y, z], dtype=np.float64)
            model.body_quat[body_id, 0:4] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

        for geom_id in range(geom_adr, geom_adr + geom_num):
            if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_BOX:
                color = rng.uniform(0.12, 0.98, size=3)
                model.geom_rgba[geom_id, 0:3] = color
                model.geom_rgba[geom_id, 3] = 1.0

        randomized += 1

    if randomized > 0:
        mujoco.mj_forward(model, data)
        print(f"[Scene] randomized {randomized} colored cubes (prefix={prefix})")
    else:
        print(f"[Scene] no cubes found for randomization (prefix={prefix})")
    return randomized


def enable_camera_debug_in_viewer(viewer, model, camera_name, cfg):
    """Draw camera frustum/frame in the main MuJoCo viewer for FOV validation."""
    if not bool(cfg.get("enable", False)):
        return

    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cam_id < 0:
        print(f"[CameraDebug] camera not found: {camera_name}")
        return

    show_frustum = bool(cfg.get("show_frustum", True))
    show_frame = bool(cfg.get("show_frame", True))

    if show_frustum:
        viewer.opt.flags[int(mujoco.mjtVisFlag.mjVIS_CAMERA)] = True
    if show_frame:
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_CAMERA

    print(
        f"[CameraDebug] enabled in main viewer: camera={camera_name}, "
        f"show_frustum={show_frustum}, show_frame={show_frame}"
    )


def apply_head_camera_view(viewer, model, camera_name, cfg):
    """Switch main viewer to the robot head camera defined in XML."""
    if not bool(cfg.get("enable", False)):
        return -1

    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cam_id < 0:
        print(f"[HeadCameraView] camera not found: {camera_name}")
        return -1

    viewer.cam.type = int(mujoco.mjtCamera.mjCAMERA_FIXED)
    viewer.cam.fixedcamid = cam_id

    if bool(cfg.get("print_definition", True)):
        body_id = int(model.cam_bodyid[cam_id])
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) if body_id >= 0 else "world"
        cam_pos = np.array(model.cam_pos[cam_id], dtype=np.float64)
        cam_quat = np.array(model.cam_quat[cam_id], dtype=np.float64)
        cam_fovy = float(model.cam_fovy[cam_id])
        print(
            "[HeadCameraView] loaded from XML: "
            f"name={camera_name}, id={cam_id}, body={body_name}, "
            f"pos={np.array2string(cam_pos, precision=5)}, "
            f"quat={np.array2string(cam_quat, precision=5)}, "
            f"fovy_deg={cam_fovy:.5f}"
        )
    return cam_id


def lock_viewer_to_fixed_camera(viewer, cam_id):
    """Keep viewer camera fixed (prevents fallback to free camera)."""
    if cam_id < 0:
        return
    viewer.cam.type = int(mujoco.mjtCamera.mjCAMERA_FIXED)
    viewer.cam.fixedcamid = int(cam_id)


def build_camera_overlay_info(model, camera_name, width, height):
    """Extract camera definition from model for preview overlay."""
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cam_id < 0:
        return None

    fovy_deg = float(model.cam_fovy[cam_id])
    fovy_rad = np.deg2rad(fovy_deg)
    aspect = float(width) / max(1.0, float(height))
    fovx_rad = 2.0 * np.arctan(np.tan(fovy_rad * 0.5) * aspect)
    fovx_deg = float(np.rad2deg(fovx_rad))

    body_id = int(model.cam_bodyid[cam_id])
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) if body_id >= 0 else "world"
    cam_pos = np.array(model.cam_pos[cam_id], dtype=np.float64)
    cam_quat = np.array(model.cam_quat[cam_id], dtype=np.float64)
    return {
        "id": cam_id,
        "name": camera_name,
        "body": body_name,
        "fovy_deg": fovy_deg,
        "fovx_deg": fovx_deg,
        "pos": cam_pos,
        "quat": cam_quat,
    }


def draw_camera_overlay(frame_bgr, overlay_info, cfg, cv2_mod):
    """Draw camera FOV/reticle overlay in offscreen preview window."""
    if overlay_info is None or not bool(cfg.get("draw_overlay", True)):
        return frame_bgr

    h, w = frame_bgr.shape[:2]
    cx, cy = w // 2, h // 2
    color_main = (0, 255, 255)   # yellow
    color_text = (40, 230, 40)   # green
    color_box = (255, 180, 0)    # cyan-ish

    # Frame border and center crosshair.
    cv2_mod.rectangle(frame_bgr, (2, 2), (w - 3, h - 3), color_box, 1)
    cv2_mod.line(frame_bgr, (cx - 18, cy), (cx + 18, cy), color_main, 1, cv2_mod.LINE_AA)
    cv2_mod.line(frame_bgr, (cx, cy - 18), (cx, cy + 18), color_main, 1, cv2_mod.LINE_AA)
    cv2_mod.circle(frame_bgr, (cx, cy), 3, color_main, -1, cv2_mod.LINE_AA)

    # Draw a simple FOV wedge from near-bottom center.
    origin = (cx, int(h * 0.90))
    ray_len = int(h * 0.62)
    half_fovx_rad = np.deg2rad(float(overlay_info["fovx_deg"]) * 0.5)
    dx = int(np.tan(half_fovx_rad) * ray_len)
    dx = int(np.clip(dx, 10, int(w * 0.48)))
    top_y = max(8, origin[1] - ray_len)
    left_pt = (max(6, cx - dx), top_y)
    right_pt = (min(w - 6, cx + dx), top_y)
    cv2_mod.line(frame_bgr, origin, left_pt, color_main, 1, cv2_mod.LINE_AA)
    cv2_mod.line(frame_bgr, origin, right_pt, color_main, 1, cv2_mod.LINE_AA)

    # Camera definition text from XML.
    line1 = (
        f"cam={overlay_info['name']} body={overlay_info['body']} "
        f"fovy={overlay_info['fovy_deg']:.2f}deg fovx={overlay_info['fovx_deg']:.2f}deg"
    )
    line2 = (
        f"pos={np.array2string(overlay_info['pos'], precision=3, suppress_small=True)} "
        f"quat={np.array2string(overlay_info['quat'], precision=3, suppress_small=True)}"
    )
    cv2_mod.putText(frame_bgr, line1, (8, 22), cv2_mod.FONT_HERSHEY_SIMPLEX, 0.50, color_text, 1, cv2_mod.LINE_AA)
    cv2_mod.putText(frame_bgr, line2, (8, 44), cv2_mod.FONT_HERSHEY_SIMPLEX, 0.45, color_text, 1, cv2_mod.LINE_AA)
    return frame_bgr


if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    mujoco_yaml_path = os.path.join(current_dir, "config", "mujoco.yaml")
    with open(mujoco_yaml_path, "r", encoding="utf-8") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        xml_path = os.path.join(PROJECT_ROOT, config["xml_path"])
        simulation_dt = config["simulation_dt"]
        control_decimation = config["control_decimation"]

    safety_yaml_path = os.path.join(current_dir, "config", "safety.yaml")
    safety_cfg = load_safety_config(safety_yaml_path)

    head_cam_cfg = config.get("head_camera_stream", {})
    head_cam_enable = bool(head_cam_cfg.get("enable", False))
    head_cam_fps = float(head_cam_cfg.get("fps", 20))
    head_cam_every_n_steps = max(1, int(round(1.0 / max(simulation_dt * head_cam_fps, 1e-6))))

    offscreen_cfg = config.get("offscreen_renderer", {})
    head_camera_view_cfg = config.get("head_camera_view", {})
    head_camera_name = str(head_camera_view_cfg.get("camera_name", "head_rgba_camera"))
    head_camera_lock = bool(head_camera_view_cfg.get("lock_view", True))
    offscreen_enable = bool(offscreen_cfg.get("enable", False))
    offscreen_fps = float(offscreen_cfg.get("fps", 20))
    offscreen_every_n_steps = max(1, int(round(1.0 / max(simulation_dt * offscreen_fps, 1e-6))))
    camera_debug_cfg = config.get("camera_debug", {})
    camera_debug_name = str(
        camera_debug_cfg.get(
            "camera_name",
            offscreen_cfg.get("camera_name", head_cam_cfg.get("camera_name", head_camera_name)),
        )
    )
    random_blocks_cfg = config.get("random_blocks", {})
    offscreen_preview_enable = bool(offscreen_cfg.get("preview_window", False))
    offscreen_preview_title = str(offscreen_cfg.get("preview_title", "Offscreen Camera Preview"))
    offscreen_draw_overlay = bool(offscreen_cfg.get("draw_overlay", True))
    offscreen_cv2 = None
    offscreen_overlay_info = None
    if offscreen_preview_enable:
        try:
            import cv2 as _cv2
            offscreen_cv2 = _cv2
        except Exception as e:
            offscreen_preview_enable = False
            print(f"[OffscreenPreview] disabled: cv2 import failed: {e!r}")

    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    randomize_cube_obstacles(m, d, random_blocks_cfg)
    m.opt.timestep = simulation_dt
    mj_per_step_duration = simulation_dt * control_decimation
    num_joints = m.nu
    qj_slice = slice(7, 7 + num_joints)
    dqj_slice = slice(6, 6 + num_joints)
    policy_output_action = np.zeros(num_joints, dtype=np.float32)
    kps = np.zeros(num_joints, dtype=np.float32)
    kds = np.zeros(num_joints, dtype=np.float32)
    sim_counter = 0

    state_cmd = StateAndCmd(num_joints)
    policy_output = PolicyOutput(num_joints)
    FSM_controller = FSM(state_cmd, policy_output)
    safety = SafetyFilter(num_joints, safety_cfg)
    command_gate = HoldToConfirm(safety_cfg.command_hold_frames)

    head_cam_stream = None
    if head_cam_enable:
        if HeadCameraStreamServer is None:
            print(f"[CameraStream] disabled: stream module import failed: {_head_camera_stream_import_error!r}")
        else:
            try:
                head_cam_stream = HeadCameraStreamServer(
                    m,
                    d,
                    camera_name=head_cam_cfg.get("camera_name", head_camera_name),
                    host=head_cam_cfg.get("host", "0.0.0.0"),
                    port=int(head_cam_cfg.get("port", 18080)),
                    width=int(head_cam_cfg.get("width", 640)),
                    height=int(head_cam_cfg.get("height", 480)),
                    jpeg_quality=int(head_cam_cfg.get("jpeg_quality", 80)),
                )
                head_cam_stream.start()
            except Exception as e:
                head_cam_stream = None
                print(f"[CameraStream] failed to start: {e}")

    offscreen_renderer = None
    if offscreen_enable:
        if OffscreenCameraRenderer is None:
            print(f"[Offscreen] disabled: offscreen module import failed: {_offscreen_renderer_import_error!r}")
        else:
            try:
                offscreen_renderer = OffscreenCameraRenderer(
                    model=m,
                    data=d,
                    camera_name=offscreen_cfg.get("camera_name", head_camera_name),
                    resolution=(
                        int(offscreen_cfg.get("width", 640)),
                        int(offscreen_cfg.get("height", 480)),
                    ),
                )
                print(
                    f"[Offscreen] started: camera={offscreen_cfg.get('camera_name', head_camera_name)}, "
                    f"res={int(offscreen_cfg.get('width', 640))}x{int(offscreen_cfg.get('height', 480))}, "
                    f"fps={offscreen_fps}, backend={getattr(offscreen_renderer, '_backend', 'unknown')}, "
                    f"preview_window={offscreen_preview_enable}"
                )
                if offscreen_preview_enable:
                    offscreen_overlay_info = build_camera_overlay_info(
                        m,
                        offscreen_cfg.get("camera_name", head_camera_name),
                        int(offscreen_cfg.get("width", 640)),
                        int(offscreen_cfg.get("height", 480)),
                    )
                    if offscreen_overlay_info is not None and offscreen_draw_overlay:
                        print(
                            "[OffscreenOverlay] draw camera view: "
                            f"fovy={offscreen_overlay_info['fovy_deg']:.3f}deg, "
                            f"fovx={offscreen_overlay_info['fovx_deg']:.3f}deg"
                        )
            except Exception as e:
                offscreen_renderer = None
                print(f"[Offscreen] failed to start: {e}")

    joystick = JoyStick()
    Running = True
    try:
        with mujoco.viewer.launch_passive(m, d) as viewer:
            head_camera_id = apply_head_camera_view(viewer, m, head_camera_name, head_camera_view_cfg)
            enable_camera_debug_in_viewer(viewer, m, camera_debug_name, camera_debug_cfg)
            sim_start_time = time.time()
            while viewer.is_running() and Running:
                try:
                    if head_camera_lock:
                        lock_viewer_to_fixed_camera(viewer, head_camera_id)
                    if(joystick.is_button_pressed(JoystickButton.SELECT)):
                        Running = False

                    joystick.update()
                    if joystick.is_button_released(JoystickButton.L3):
                        state_cmd.skill_cmd = FSMCommand.PASSIVE
                    if joystick.is_button_released(JoystickButton.UP):
                        state_cmd.skill_cmd = FSMCommand.PAUSE
                    if command_gate.trigger("POS_RESET", joystick.is_button_pressed(JoystickButton.START)):
                        state_cmd.skill_cmd = FSMCommand.POS_RESET
                    if command_gate.trigger(
                        "LOCO",
                        joystick.is_button_pressed(JoystickButton.A) and joystick.is_button_pressed(JoystickButton.R1),
                    ):
                        state_cmd.skill_cmd = FSMCommand.LOCO
                    if command_gate.trigger(
                        "SKILL_1",
                        joystick.is_button_pressed(JoystickButton.X) and joystick.is_button_pressed(JoystickButton.R1),
                    ):
                        state_cmd.skill_cmd = FSMCommand.SKILL_1
                    if command_gate.trigger(
                        "SKILL_2",
                        joystick.is_button_pressed(JoystickButton.Y) and joystick.is_button_pressed(JoystickButton.R1),
                    ):
                        state_cmd.skill_cmd = FSMCommand.SKILL_2
                    if joystick.is_button_released(JoystickButton.B) and joystick.is_button_pressed(JoystickButton.R1):
                        state_cmd.skill_cmd = FSMCommand.SKILL_3
                    if command_gate.trigger(
                        "SKILL_4",
                        joystick.is_button_pressed(JoystickButton.Y) and joystick.is_button_pressed(JoystickButton.L1),
                    ):
                        state_cmd.skill_cmd = FSMCommand.SKILL_4
                    if joystick.is_button_released(JoystickButton.B) and joystick.is_button_pressed(JoystickButton.L1):
                        state_cmd.skill_cmd = FSMCommand.SKILL_5
                    if command_gate.trigger(
                        "SKILL_6",
                        joystick.is_button_pressed(JoystickButton.X) and joystick.is_button_pressed(JoystickButton.L1),
                    ):
                        state_cmd.skill_cmd = FSMCommand.SKILL_6
                    if command_gate.trigger(
                        "SKILL_7",
                        joystick.is_button_pressed(JoystickButton.A) and joystick.is_button_pressed(JoystickButton.L1),
                    ):
                        state_cmd.skill_cmd = FSMCommand.SKILL_7

                    state_cmd.vel_cmd[0] = -joystick.get_axis_value(1)
                    state_cmd.vel_cmd[1] = -joystick.get_axis_value(0)
                    state_cmd.vel_cmd[2] = -joystick.get_axis_value(3)

                    step_start = time.time()

                    raw_tau = pd_control(
                        policy_output_action,
                        d.qpos[qj_slice],
                        kps,
                        np.zeros_like(kps),
                        d.qvel[dqj_slice],
                        kds,
                    )
                    fallback_tau = -safety_cfg.damping_kd * np.asarray(d.qvel[dqj_slice], dtype=np.float32)
                    tau = sanitize_ctrl(raw_tau, m, fallback=fallback_tau)
                    if (not np.isfinite(raw_tau).all()) or np.max(np.abs(raw_tau)) > 1e4:
                        print("[Safety] abnormal torque detected; sanitized before applying ctrl.")
                    if safety_cfg.dry_run:
                        d.ctrl[:] = 0
                    else:
                        d.ctrl[:] = tau
                    # Apply mouse drag perturbations from the viewer in passive mode.
                    if hasattr(viewer, "pert"):
                        d.xfrc_applied[:] = 0
                        mujoco.mjv_applyPerturbForce(m, d, viewer.pert)
                    mujoco.mj_step(m, d)
                    sim_counter += 1

                    if head_cam_stream is not None and (sim_counter % head_cam_every_n_steps == 0):
                        head_cam_stream.update()
                    if offscreen_renderer is not None and (sim_counter % offscreen_every_n_steps == 0):
                        frame_rgba = offscreen_renderer.render_rgba()
                        if offscreen_preview_enable and offscreen_cv2 is not None:
                            frame_bgr = offscreen_cv2.cvtColor(frame_rgba[:, :, :3], offscreen_cv2.COLOR_RGB2BGR)
                            if offscreen_draw_overlay:
                                frame_bgr = draw_camera_overlay(
                                    frame_bgr, offscreen_overlay_info, offscreen_cfg, offscreen_cv2
                                )
                            offscreen_cv2.imshow(offscreen_preview_title, frame_bgr)
                            offscreen_cv2.waitKey(1)

                    if sim_counter % control_decimation == 0:

                        qj = d.qpos[qj_slice]
                        dqj = d.qvel[dqj_slice]
                        quat = d.qpos[3:7]

                        omega = d.qvel[3:6]
                        gravity_orientation = get_gravity_orientation(quat)

                        state_cmd.q = qj.copy()
                        state_cmd.dq = dqj.copy()
                        state_cmd.gravity_ori = gravity_orientation.copy()
                        state_cmd.base_quat = quat.copy()
                        state_cmd.ang_vel = omega.copy()

                        FSM_controller.run()
                        policy_output_action = policy_output.actions.copy()
                        kps = policy_output.kps.copy()
                        kds = policy_output.kds.copy()

                        policy_output_action, kps, kds, force_damping = safety.filter_actions(
                            policy_output_action, kps, kds
                        )
                        if force_damping:
                            policy_output_action = d.qpos[qj_slice].copy()
                            kps = np.zeros_like(kps)
                            kds = np.ones_like(kds) * safety_cfg.damping_kd
                        if safety_cfg.dry_run:
                            kps = np.zeros_like(kps)
                            kds = np.zeros_like(kds)
                except ValueError as e:
                    print(str(e))

                viewer.sync()
                time_until_next_step = m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
    except KeyboardInterrupt:
        print("[Main] interrupted by user (Ctrl+C).")
    finally:
        if head_cam_stream is not None:
            head_cam_stream.close()
        if offscreen_renderer is not None:
            offscreen_renderer.close()
        if offscreen_preview_enable and offscreen_cv2 is not None:
            try:
                offscreen_cv2.destroyWindow(offscreen_preview_title)
            except Exception:
                pass
