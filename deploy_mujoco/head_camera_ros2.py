import time

import mujoco
import numpy as np


class HeadCameraRos2Publisher:
    """Publish MuJoCo head-camera frames to ROS2 topics.

    Publishes:
      - RGB8  : topic_rgb
      - RGBA8 : topic_rgba
    """

    def __init__(
        self,
        model,
        data,
        camera_name="head_rgba_camera",
        width=640,
        height=480,
        topic_rgb="/z1/head_camera/rgb",
        topic_rgba="/z1/head_camera/rgba",
        frame_id="head_rgba_camera",
        node_name="z1_head_camera_publisher",
        qos_depth=5,
    ):
        # Lazy import so deploy script can run without ROS2 installed.
        import rclpy
        from rclpy.qos import QoSProfile
        from sensor_msgs.msg import Image

        self.rclpy = rclpy
        self.Image = Image

        self._did_init = False
        if not self.rclpy.ok():
            self.rclpy.init(args=None)
            self._did_init = True

        qos = QoSProfile(depth=int(qos_depth))
        self.node = self.rclpy.create_node(node_name)
        self.pub_rgb = self.node.create_publisher(self.Image, topic_rgb, qos)
        self.pub_rgba = self.node.create_publisher(self.Image, topic_rgba, qos)

        self.model = model
        self.data = data
        self.camera_name = camera_name
        self.width = int(width)
        self.height = int(height)
        self.frame_id = frame_id

        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if camera_id < 0:
            raise ValueError(f"camera '{camera_name}' not found in model")

        self.renderer = mujoco.Renderer(model, width=self.width, height=self.height)
        self.seq = 0

        print(
            f"[CameraROS2] publish {topic_rgb} (rgb8), {topic_rgba} (rgba8), "
            f"camera={camera_name}, size={self.width}x{self.height}"
        )

    def _to_msg(self, img: np.ndarray, encoding: str):
        msg = self.Image()
        now = self.node.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.height = int(img.shape[0])
        msg.width = int(img.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = int(img.shape[1] * img.shape[2])
        msg.data = img.tobytes()
        return msg

    def publish(self):
        self.renderer.update_scene(self.data, camera=self.camera_name)
        rgb = self.renderer.render()  # HxWx3 RGB

        alpha = np.full((rgb.shape[0], rgb.shape[1], 1), 255, dtype=np.uint8)
        rgba = np.concatenate([rgb, alpha], axis=2)

        msg_rgb = self._to_msg(rgb, "rgb8")
        msg_rgba = self._to_msg(rgba, "rgba8")

        self.pub_rgb.publish(msg_rgb)
        self.pub_rgba.publish(msg_rgba)
        self.rclpy.spin_once(self.node, timeout_sec=0.0)
        self.seq += 1

    def close(self):
        try:
            self.renderer.close()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if self._did_init:
            try:
                self.rclpy.shutdown()
            except Exception:
                pass
