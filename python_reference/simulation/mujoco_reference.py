import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from shared.path_config import PROJECT_ROOT

import os
os.environ.setdefault("__GL_SYNC_TO_VBLANK", "0")
os.environ.setdefault("vblank_mode", "0")
import time
import mujoco.viewer
import mujoco
import numpy as np
import yaml
from shared.ctrlcomp import *
from fsm.machine import *
from shared.utils import get_gravity_orientation
from shared.joystick import JoyStick, JoystickButton
from shared.safety import load_safety_config, SafetyFilter, HoldToConfirm

try:
    from simulation.head_camera_stream import HeadCameraStreamServer
except Exception:
    HeadCameraStreamServer = None

try:
    from simulation.head_camera_ros2 import HeadCameraRos2Publisher
except Exception:
    HeadCameraRos2Publisher = None


class NullJoyStick:
    def update(self):
        return None

    def is_button_pressed(self, _button_id):
        return False

    def is_button_released(self, _button_id):
        return False

    def get_axis_value(self, _axis_id):
        return 0.0


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd


def sanitize_ctrl(ctrl, model, fallback=None, ctrl_limit=None):
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

    if ctrl_limit is not None:
        limit = np.asarray(ctrl_limit, dtype=np.float32).reshape(-1)
        if limit.size == out.size and np.any(limit > 0.0):
            limit = np.maximum(limit, 0.0)
            return np.clip(out, -limit, limit).astype(np.float32)

    ctrl_range = getattr(model, "actuator_ctrlrange", None)
    if ctrl_range is not None and ctrl_range.shape[0] == out.size:
        lo = ctrl_range[:, 0]
        hi = ctrl_range[:, 1]
        # Some models may leave ctrlrange unset (lo==hi==0), fallback in that case.
        if np.any(hi > lo):
            return np.clip(out, lo, hi).astype(np.float32)

    return np.clip(out, -300.0, 300.0).astype(np.float32)


def resolve_project_path(path_value):
    if not path_value:
        return None
    path_value = os.path.expanduser(os.path.expandvars(str(path_value)))
    if os.path.isabs(path_value):
        return path_value
    return os.path.join(PROJECT_ROOT, path_value)


def load_initial_joint_targets(yaml_path, num_joints):
    """Load lab-order BeyondMimic joint values and map them into MuJoCo actuator order."""
    if not yaml_path or not os.path.isfile(yaml_path):
        return None, None, None, None
    with open(yaml_path, "r") as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader) or {}

    mj2lab = np.array(cfg.get("mj2lab", []), dtype=np.int32)
    default_lab = np.array(cfg.get("default_angles_lab", []), dtype=np.float32)
    kp_lab = np.array(cfg.get("kp_lab", []), dtype=np.float32)
    kd_lab = np.array(cfg.get("kd_lab", []), dtype=np.float32)
    tau_lab = np.array(cfg.get("tau_limit", []), dtype=np.float32)

    if mj2lab.size == 0 or default_lab.size == 0:
        return None, None, None, None

    default_mj = np.zeros(num_joints, dtype=np.float32)
    kp_mj = np.zeros(num_joints, dtype=np.float32)
    kd_mj = np.zeros(num_joints, dtype=np.float32)
    tau_mj = np.zeros(num_joints, dtype=np.float32)

    for lab_idx, mj_idx in enumerate(mj2lab):
        if not (0 <= mj_idx < num_joints):
            continue
        if lab_idx < default_lab.size:
            default_mj[mj_idx] = default_lab[lab_idx]
        if lab_idx < kp_lab.size:
            kp_mj[mj_idx] = kp_lab[lab_idx]
        if lab_idx < kd_lab.size:
            kd_mj[mj_idx] = kd_lab[lab_idx]
        if lab_idx < tau_lab.size:
            tau_mj[mj_idx] = tau_lab[lab_idx]
    return default_mj, kp_mj, kd_mj, tau_mj


def parse_initial_command(name):
    if not name:
        return FSMCommand.INVALID
    key = str(name).strip().upper()
    return getattr(FSMCommand, key, FSMCommand.INVALID)


def parse_bool(value, default=False):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "y", "on"}
    return default


def actuator_joint_indices(model):
    """Return qpos/qvel indices in actuator order, excluding the floating base offsets."""
    qpos_idx = np.zeros(model.nu, dtype=np.int32)
    qvel_idx = np.zeros(model.nu, dtype=np.int32)
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        qpos_idx[actuator_id] = int(model.jnt_qposadr[joint_id] - 7)
        qvel_idx[actuator_id] = int(model.jnt_dofadr[joint_id] - 6)
    return qpos_idx, qvel_idx


def get_actuator_order_state(data, qpos_idx, qvel_idx):
    return data.qpos[7 + qpos_idx].copy(), data.qvel[6 + qvel_idx].copy()


def fit_vector(values, size, fill=0.0):
    out = np.full(size, fill, dtype=np.float32)
    if values is None:
        return out
    arr = np.asarray(values, dtype=np.float32).reshape(-1)
    n = min(size, arr.size)
    if n > 0:
        out[:n] = arr[:n]
    return out


DEFAULT_GHOST_BODY_NAMES = (
    "pelvis",
    "torso_link",
    "head_link",
    "left_hip_pitch_link",
    "left_hip_yaw_link",
    "left_knee_link",
    "left_ankle_roll_link",
    "left_toe_link",
    "right_hip_pitch_link",
    "right_hip_yaw_link",
    "right_knee_link",
    "right_ankle_roll_link",
    "right_toe_link",
    "left_shoulder_pitch_link",
    "left_shoulder_yaw_link",
    "left_elbow_link",
    "left_wrist_yaw_link",
    "left_hand_palm_link",
    "right_shoulder_pitch_link",
    "right_shoulder_yaw_link",
    "right_elbow_link",
    "right_wrist_yaw_link",
    "right_hand_palm_link",
)


def set_ghost_pose_from_policy(ghost_data, sim_data, policy, qpos_actuator_idx, num_joints):
    """Update ghost qpos from the active mimic policy reference joint pose."""
    ref_joint_pos = getattr(policy, "ref_joint_pos", None)
    mj2lab = getattr(policy, "mj2lab", None)
    if ref_joint_pos is None or mj2lab is None:
        return False

    ref_lab = fit_vector(ref_joint_pos, num_joints)
    mj2lab = np.asarray(mj2lab, dtype=np.int32).reshape(-1)
    if mj2lab.size == 0:
        return False

    ref_mj = np.zeros(num_joints, dtype=np.float32)
    for lab_idx, mj_idx in enumerate(mj2lab):
        if lab_idx < ref_lab.size and 0 <= mj_idx < num_joints:
            ref_mj[mj_idx] = ref_lab[lab_idx]

    ghost_data.qpos[:] = sim_data.qpos
    ghost_data.qvel[:] = 0.0
    ghost_data.qpos[7 + qpos_actuator_idx] = ref_mj
    return True


def resolve_ghost_body_ids(model, body_names):
    ids = []
    seen = set()
    for name in body_names or DEFAULT_GHOST_BODY_NAMES:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, str(name))
        if body_id >= 0 and body_id not in seen:
            seen.add(body_id)
            ids.append(body_id)
    return np.asarray(ids, dtype=np.int32)


def build_ghost_body_links(model, body_ids):
    selected = {int(body_id) for body_id in body_ids}
    links = []
    for child_id in body_ids:
        child_id = int(child_id)
        parent_id = int(model.body_parentid[child_id])
        while parent_id > 0:
            if parent_id in selected:
                links.append((parent_id, child_id))
                break
            parent_id = int(model.body_parentid[parent_id])
    return links


def _next_user_scene_geom(user_scn):
    if user_scn.ngeom >= len(user_scn.geoms):
        return None
    geom = user_scn.geoms[user_scn.ngeom]
    user_scn.ngeom += 1
    return geom


def update_ghost_skeleton_scene(
    model,
    ghost_data,
    viewer,
    body_ids,
    body_links,
    rgba,
    marker_radius,
    link_width,
    draw_markers=True,
    use_line_links=False,
):
    """Render a light reference outline/skeleton instead of a second mesh robot."""
    user_scn = getattr(viewer, "user_scn", None)
    if user_scn is None:
        return

    mujoco.mj_forward(model, ghost_data)
    user_scn.ngeom = 0

    rgba = np.asarray(rgba, dtype=np.float32).reshape(-1)
    if rgba.size != 4:
        rgba = np.array([0.0, 0.85, 0.38, 0.42], dtype=np.float32)
    identity = np.eye(3, dtype=np.float64).reshape(-1)
    zero_size = np.zeros(3, dtype=np.float64)
    zero_pos = np.zeros(3, dtype=np.float64)

    link_geom_type = int(mujoco.mjtGeom.mjGEOM_LINE if use_line_links else mujoco.mjtGeom.mjGEOM_CAPSULE)
    for parent_id, child_id in body_links:
        start = np.asarray(ghost_data.xpos[parent_id], dtype=np.float64)
        end = np.asarray(ghost_data.xpos[child_id], dtype=np.float64)
        if np.linalg.norm(end - start) < 1e-5:
            continue
        geom = _next_user_scene_geom(user_scn)
        if geom is None:
            return
        mujoco.mjv_initGeom(
            geom,
            link_geom_type,
            zero_size,
            zero_pos,
            identity,
            rgba,
        )
        mujoco.mjv_connector(
            geom,
            link_geom_type,
            float(link_width),
            start,
            end,
        )
        geom.rgba[:] = rgba

    if not draw_markers:
        return

    sphere_size = np.array([marker_radius, marker_radius, marker_radius], dtype=np.float64)
    for body_id in body_ids:
        geom = _next_user_scene_geom(user_scn)
        if geom is None:
            return
        mujoco.mjv_initGeom(
            geom,
            int(mujoco.mjtGeom.mjGEOM_SPHERE),
            sphere_size,
            np.asarray(ghost_data.xpos[int(body_id)], dtype=np.float64),
            identity,
            rgba,
        )


def update_ghost_scene(model, ghost_data, viewer, rgba):
    """Render a transparent reference robot through viewer.user_scn."""
    user_scn = getattr(viewer, "user_scn", None)
    if user_scn is None:
        return
    mujoco.mj_forward(model, ghost_data)
    mujoco.mjv_updateScene(
        model,
        ghost_data,
        viewer.opt,
        None,
        viewer.cam,
        int(mujoco.mjtCatBit.mjCAT_DYNAMIC),
        user_scn,
    )
    rgba = np.asarray(rgba, dtype=np.float32).reshape(-1)
    if rgba.size != 4:
        rgba = np.array([0.1, 0.6, 1.0, 0.28], dtype=np.float32)
    for geom_id in range(user_scn.ngeom):
        user_scn.geoms[geom_id].rgba[:] = rgba


def clear_ghost_scene(viewer):
    user_scn = getattr(viewer, "user_scn", None)
    if user_scn is not None:
        user_scn.ngeom = 0


def resolve_contact_geom_ids(model, body_keywords):
    keywords = [str(v).lower() for v in (body_keywords or []) if str(v).strip()]
    if not keywords:
        return set(range(model.ngeom))
    geom_ids = set()
    for geom_id in range(model.ngeom):
        body_id = int(model.geom_bodyid[geom_id])
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        body_name = body_name.lower()
        if any(keyword in body_name for keyword in keywords):
            geom_ids.add(geom_id)
    return geom_ids


def correct_ground_penetration(model, data, floor_geom_id, contact_geom_ids, max_penetration):
    if floor_geom_id < 0 or max_penetration is None or max_penetration < 0:
        return 0.0
    min_dist = 0.0
    for contact_id in range(data.ncon):
        contact = data.contact[contact_id]
        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)
        if geom1 == floor_geom_id and geom2 in contact_geom_ids:
            min_dist = min(min_dist, float(contact.dist))
        elif geom2 == floor_geom_id and geom1 in contact_geom_ids:
            min_dist = min(min_dist, float(contact.dist))

    allowed_dist = -float(max_penetration)
    if min_dist >= allowed_dist:
        return 0.0

    correction = allowed_dist - min_dist
    data.qpos[2] += correction
    if data.qvel.shape[0] > 2 and data.qvel[2] < 0.0:
        data.qvel[2] = 0.0
    mujoco.mj_forward(model, data)
    return correction


if __name__ == "__main__":
    mujoco_yaml_path = os.path.join(PROJECT_ROOT, "configs", "simulation", "mujoco.yaml")
    with open(mujoco_yaml_path, "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        xml_path = os.path.join(PROJECT_ROOT, config["xml_path"])
        simulation_dt = config["simulation_dt"]
        control_decimation = config["control_decimation"]
        render_fps = float(config.get("render_fps", 60))
        perf_log_interval_s = float(config.get("perf_log_interval_s", 0.0))
        initial_pose_yaml = resolve_project_path(config.get("initial_pose_yaml", None))
        initial_command = parse_initial_command(config.get("initial_command", ""))
        abnormal_log_interval = max(1, int(config.get("abnormal_torque_log_interval_steps", 100)))
        ground_correction_cfg = config.get("ground_penetration_correction", {}) or {}
        ghost_cfg = config.get("ghost", {}) or {}

    safety_yaml_path = os.path.join(PROJECT_ROOT, "configs", "simulation", "safety.yaml")
    safety_cfg = load_safety_config(safety_yaml_path)

    head_cam_cfg = config.get("head_camera_stream", {})
    head_cam_enable = bool(head_cam_cfg.get("enable", False))
    head_cam_fps = float(head_cam_cfg.get("fps", 20))
    head_cam_every_n_steps = max(1, int(round(1.0 / max(simulation_dt * head_cam_fps, 1e-6))))

    ros2_cam_cfg = config.get("head_camera_ros2", {})
    ros2_cam_enable = bool(ros2_cam_cfg.get("enable", False))
    ros2_cam_fps = float(ros2_cam_cfg.get("fps", 20))
    ros2_cam_every_n_steps = max(1, int(round(1.0 / max(simulation_dt * ros2_cam_fps, 1e-6))))

    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    ghost_d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt
    mj_per_step_duration = simulation_dt * control_decimation
    num_joints = m.nu
    qpos_actuator_idx, qvel_actuator_idx = actuator_joint_indices(m)
    policy_output_action = np.zeros(num_joints, dtype=np.float32)
    kps = np.zeros(num_joints, dtype=np.float32)
    kds = np.zeros(num_joints, dtype=np.float32)
    sim_counter = 0
    last_abnormal_log_step = -abnormal_log_interval
    ground_correction_enable = parse_bool(ground_correction_cfg.get("enable", False), False)
    ground_correction_floor_id = mujoco.mj_name2id(
        m,
        mujoco.mjtObj.mjOBJ_GEOM,
        str(ground_correction_cfg.get("floor_geom", "floor")),
    )
    ground_correction_geom_ids = resolve_contact_geom_ids(
        m,
        ground_correction_cfg.get("body_keywords", []),
    )
    ground_correction_max_penetration = float(ground_correction_cfg.get("max_penetration", 0.0))
    ghost_enable = parse_bool(ghost_cfg.get("enable", False), False)
    ghost_mode = str(ghost_cfg.get("mode", "outline")).strip().lower()
    if ghost_mode not in {"outline", "skeleton", "mesh"}:
        print(f"[Ghost] unknown mode '{ghost_mode}', fallback to outline.")
        ghost_mode = "outline"
    ghost_update_fps = float(ghost_cfg.get("update_fps", 8.0 if ghost_mode in {"outline", "skeleton"} else render_fps))
    ghost_update_interval = 1.0 / max(ghost_update_fps, 1e-6)
    ghost_marker_radius = float(ghost_cfg.get("marker_radius", 0.025))
    ghost_link_width = float(ghost_cfg.get("link_width", 0.012))
    ghost_line_width = float(ghost_cfg.get("line_width", 3.0))
    ghost_draw_markers = parse_bool(ghost_cfg.get("draw_markers", ghost_mode == "skeleton"), ghost_mode == "skeleton")
    ghost_alpha = float(ghost_cfg.get("alpha", 0.28))
    ghost_rgba = np.asarray(ghost_cfg.get("rgba", [0.1, 0.6, 1.0, ghost_alpha]), dtype=np.float32)
    if ghost_rgba.size != 4:
        ghost_rgba = np.array([0.1, 0.6, 1.0, ghost_alpha], dtype=np.float32)
    ghost_rgba[3] = ghost_alpha
    ghost_body_ids = resolve_ghost_body_ids(m, ghost_cfg.get("body_names", DEFAULT_GHOST_BODY_NAMES))
    if ghost_body_ids.size == 0:
        ghost_body_ids = resolve_ghost_body_ids(m, ("pelvis",))
    ghost_body_links = build_ghost_body_links(m, ghost_body_ids)
    if ghost_enable:
        print(
            f"[Ghost] mode={ghost_mode} update_fps={ghost_update_fps:.1f} "
            f"bodies={ghost_body_ids.size} links={len(ghost_body_links)}"
        )

    init_q, init_kp, init_kd, ctrl_limit = load_initial_joint_targets(initial_pose_yaml, num_joints)
    if init_q is not None:
        d.qpos[7 + qpos_actuator_idx] = init_q
        d.qvel[:] = 0.0
        mujoco.mj_forward(m, d)
        policy_output_action = init_q.copy()
        if init_kp is not None:
            kps = init_kp.copy()
        if init_kd is not None:
            kds = init_kd.copy()
        print(f"[MuJoCo] initialized joint pose from {initial_pose_yaml}")

    state_cmd = StateAndCmd(num_joints)
    policy_output = PolicyOutput(num_joints)
    FSM_controller = FSM(state_cmd, policy_output)
    safety = SafetyFilter(num_joints, safety_cfg)
    command_gate = HoldToConfirm(safety_cfg.command_hold_frames)
    state_cmd.skill_cmd = initial_command

    head_cam_stream = None
    if head_cam_enable:
        if HeadCameraStreamServer is None:
            print("[CameraStream] disabled: stream module import failed.")
        else:
            try:
                head_cam_stream = HeadCameraStreamServer(
                    m,
                    d,
                    camera_name=head_cam_cfg.get("camera_name", "head_rgba_camera"),
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

    head_cam_ros2 = None
    if ros2_cam_enable:
        if HeadCameraRos2Publisher is None:
            print("[CameraROS2] disabled: ROS2 publisher module import failed.")
        else:
            try:
                head_cam_ros2 = HeadCameraRos2Publisher(
                    m,
                    d,
                    camera_name=ros2_cam_cfg.get("camera_name", "head_rgba_camera"),
                    width=int(ros2_cam_cfg.get("width", 640)),
                    height=int(ros2_cam_cfg.get("height", 480)),
                    topic_rgb=ros2_cam_cfg.get("topic_rgb", "/z1/head_camera/rgb"),
                    topic_rgba=ros2_cam_cfg.get("topic_rgba", "/z1/head_camera/rgba"),
                    frame_id=ros2_cam_cfg.get("frame_id", "head_rgba_camera"),
                    node_name=ros2_cam_cfg.get("node_name", "z1_head_camera_publisher"),
                    qos_depth=int(ros2_cam_cfg.get("qos_depth", 5)),
                )
            except Exception as e:
                head_cam_ros2 = None
                print(f"[CameraROS2] failed to start: {e}")

    try:
        joystick = JoyStick()
    except RuntimeError as e:
        print(f"[Joystick][WARN] {e} Falling back to neutral NullJoyStick.")
        joystick = NullJoyStick()
    Running = True
    command_input_armed = False
    command_buttons = (
        JoystickButton.SELECT,
        JoystickButton.L3,
        JoystickButton.UP,
        JoystickButton.START,
        JoystickButton.A,
        JoystickButton.B,
        JoystickButton.X,
        JoystickButton.Y,
        JoystickButton.L1,
        JoystickButton.R1,
    )
    try:
        with mujoco.viewer.launch_passive(m, d) as viewer:
            loop_start_time = time.perf_counter()
            next_step_time = loop_start_time
            last_viewer_sync_time = loop_start_time
            render_interval = 1.0 / max(render_fps, 1e-6)
            last_ghost_update_time = loop_start_time - ghost_update_interval
            ghost_scene_active = False
            last_perf_log_time = loop_start_time
            last_perf_log_step = sim_counter
            while viewer.is_running() and Running:
                try:
                    q_act, dq_act = get_actuator_order_state(d, qpos_actuator_idx, qvel_actuator_idx)
                    raw_tau = pd_control(policy_output_action, q_act, kps, np.zeros_like(kps), dq_act, kds)
                    fallback_tau = -safety_cfg.damping_kd * np.asarray(dq_act, dtype=np.float32)
                    tau = sanitize_ctrl(raw_tau, m, fallback=fallback_tau, ctrl_limit=ctrl_limit)
                    if (not np.isfinite(raw_tau).all()) or np.max(np.abs(raw_tau)) > 1e4:
                        if sim_counter - last_abnormal_log_step >= abnormal_log_interval:
                            last_abnormal_log_step = sim_counter
                            max_tau = float(np.nanmax(np.abs(raw_tau))) if raw_tau.size else 0.0
                            max_q = float(np.nanmax(np.abs(q_act))) if q_act.size else 0.0
                            max_dq = float(np.nanmax(np.abs(dq_act))) if dq_act.size else 0.0
                            print(
                                "[Safety] abnormal torque detected; sanitized before applying ctrl. "
                                f"max_tau={max_tau:.1f} max_q={max_q:.3f} max_dq={max_dq:.1f}"
                            )
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
                    if ground_correction_enable:
                        correct_ground_penetration(
                            m,
                            d,
                            ground_correction_floor_id,
                            ground_correction_geom_ids,
                            ground_correction_max_penetration,
                        )

                    if head_cam_stream is not None and (sim_counter % head_cam_every_n_steps == 0):
                        head_cam_stream.update()
                    if head_cam_ros2 is not None and (sim_counter % ros2_cam_every_n_steps == 0):
                        head_cam_ros2.publish()

                    if sim_counter % control_decimation == 0:
                        joystick.update()
                        command_buttons_idle = not any(
                            joystick.is_button_pressed(button)
                            for button in command_buttons
                        )
                        if not command_input_armed:
                            command_input_armed = command_buttons_idle
                        if command_input_armed:
                            if joystick.is_button_pressed(JoystickButton.SELECT):
                                Running = False
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

                        qj, dqj = get_actuator_order_state(d, qpos_actuator_idx, qvel_actuator_idx)
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
                            policy_output_action = qj.copy()
                            kps = np.zeros_like(kps)
                            kds = np.ones_like(kds) * safety_cfg.damping_kd
                        if safety_cfg.dry_run:
                            kps = np.zeros_like(kps)
                            kds = np.zeros_like(kds)
                except ValueError as e:
                    print(str(e))

                now = time.perf_counter()
                if now - last_viewer_sync_time >= render_interval:
                    if ghost_enable:
                        if now - last_ghost_update_time >= ghost_update_interval:
                            with viewer.lock():
                                if set_ghost_pose_from_policy(
                                    ghost_d,
                                    d,
                                    FSM_controller.cur_policy,
                                    qpos_actuator_idx,
                                    num_joints,
                                ):
                                    if ghost_mode == "mesh":
                                        update_ghost_scene(m, ghost_d, viewer, ghost_rgba)
                                    else:
                                        update_ghost_skeleton_scene(
                                            m,
                                            ghost_d,
                                            viewer,
                                            ghost_body_ids,
                                            ghost_body_links,
                                            ghost_rgba,
                                            ghost_marker_radius,
                                            ghost_line_width if ghost_mode == "outline" else ghost_link_width,
                                            ghost_draw_markers,
                                            ghost_mode == "outline",
                                        )
                                    ghost_scene_active = True
                                elif ghost_scene_active:
                                    clear_ghost_scene(viewer)
                                    ghost_scene_active = False
                            last_ghost_update_time = now
                    else:
                        if ghost_scene_active:
                            with viewer.lock():
                                clear_ghost_scene(viewer)
                            ghost_scene_active = False
                    viewer.sync()
                    last_viewer_sync_time = time.perf_counter()
                    now = last_viewer_sync_time

                if perf_log_interval_s > 0.0 and now - last_perf_log_time >= perf_log_interval_s:
                    elapsed = now - last_perf_log_time
                    steps = sim_counter - last_perf_log_step
                    sim_hz = steps / max(elapsed, 1e-9)
                    realtime_factor = sim_hz * simulation_dt
                    print(
                        f"[Perf] sim_hz={sim_hz:.1f} realtime={realtime_factor:.2f}x "
                        f"control_hz={sim_hz / max(control_decimation, 1):.1f} render_fps={render_fps:.1f}"
                    )
                    last_perf_log_time = now
                    last_perf_log_step = sim_counter

                next_step_time += simulation_dt
                now = time.perf_counter()
                if now >= next_step_time:
                    # Drop accumulated timing debt instead of running a burst of
                    # catch-up steps, which makes the robot appear fast/slow.
                    next_step_time = now
                else:
                    while True:
                        sleep_time = next_step_time - time.perf_counter()
                        if sleep_time <= 0:
                            break
                        time.sleep(min(sleep_time, 0.001))
    finally:
        if head_cam_stream is not None:
            head_cam_stream.close()
        if head_cam_ros2 is not None:
            head_cam_ros2.close()
