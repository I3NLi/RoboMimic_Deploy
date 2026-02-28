import os
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.append(str(Path(__file__).parent.parent.absolute()))
from common.path_config import PROJECT_ROOT

try:
    import glfw
except Exception:
    glfw = None


class OffscreenCameraRenderer:
    """Offscreen MuJoCo renderer.

    Preferred backend: hidden GLFW OpenGL context.
    Fallback backend: mujoco.Renderer (works when glfw is unavailable).
    """

    def __init__(self, model, data, camera_name="head_rgba_camera", resolution=(640, 480)):
        self.model = model
        self.data = data
        self.camera_name = camera_name
        self.width = int(resolution[0])
        self.height = int(resolution[1])
        self.window = None
        self._glfw_ok = False
        self._backend = "mujoco_renderer"
        self.renderer = None
        self.context = None
        self.scene = None

        if glfw is not None:
            if not glfw.init():
                raise RuntimeError("glfw.init() failed")
            self._glfw_ok = True

            # Hidden window: provide an OpenGL context for offscreen rendering.
            glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
            self.window = glfw.create_window(self.width, self.height, "Offscreen", None, None)
            if self.window is None:
                self.close()
                raise RuntimeError("glfw.create_window() failed")
            glfw.make_context_current(self.window)

            self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
            self.context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value)
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.context)

            self.option = mujoco.MjvOption()
            self.perturb = mujoco.MjvPerturb()
            self.viewport = mujoco.MjrRect(0, 0, self.width, self.height)

            self.camera_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name
            )
            if self.camera_id < 0:
                self.close()
                raise ValueError(f"camera '{self.camera_name}' not found in model")

            self.camera = mujoco.MjvCamera()
            self.camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.camera.fixedcamid = self.camera_id
            self._backend = "glfw_offscreen"
            return

        # glfw unavailable: fallback to mujoco.Renderer.
        self.renderer = mujoco.Renderer(self.model, width=self.width, height=self.height)
        camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name)
        if camera_id < 0:
            self.close()
            raise ValueError(f"camera '{self.camera_name}' not found in model")

    def render_rgba(self):
        """Render one RGBA frame bound to the configured fixed camera."""
        if self._backend == "glfw_offscreen":
            glfw.make_context_current(self.window)
            mujoco.mjv_updateScene(
                self.model,
                self.data,
                self.option,
                self.perturb,
                self.camera,
                mujoco.mjtCatBit.mjCAT_ALL.value,
                self.scene,
            )
            mujoco.mjr_render(self.viewport, self.scene, self.context)
            rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            depth = np.zeros((self.height, self.width), dtype=np.float32)
            mujoco.mjr_readPixels(rgb, depth, self.viewport, self.context)
            # OpenGL frame buffer origin is bottom-left.
            rgb = np.flipud(rgb)
        else:
            self.renderer.update_scene(self.data, camera=self.camera_name)
            rgb = self.renderer.render()

        alpha = np.full((self.height, self.width, 1), 255, dtype=np.uint8)
        return np.concatenate((rgb, alpha), axis=2)

    def close(self):
        if self.renderer is not None:
            try:
                self.renderer.close()
            except Exception:
                pass
            self.renderer = None

        if getattr(self, "context", None) is not None:
            try:
                self.context.free()
            except Exception:
                pass
            self.context = None

        if self.window is not None:
            try:
                glfw.destroy_window(self.window)
            except Exception:
                pass
            self.window = None

        if self._glfw_ok:
            try:
                glfw.terminate()
            except Exception:
                pass
            self._glfw_ok = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


def _default_xml_path():
    return os.path.join(PROJECT_ROOT, "g1_description", "scene.xml")


if __name__ == "__main__":
    xml_path = _default_xml_path()
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    with OffscreenCameraRenderer(
        model=model,
        data=data,
        camera_name="head_rgba_camera",
        resolution=(640, 480),
    ) as renderer:
        for _ in range(5):
            mujoco.mj_step(model, data)
        frame_rgba = renderer.render_rgba()
        print(f"Offscreen frame shape: {frame_rgba.shape}, dtype={frame_rgba.dtype}")
