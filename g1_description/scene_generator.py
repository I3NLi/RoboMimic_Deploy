"""MuJoCo scene generator.

Generates scene_withcamera.xml with a configurable number of randomly
placed boxes.  Each box uses a **single <geom>** so the collision geometry
and the render geometry are identical (same size, same shape).
pos.z == size.z (half-height) ensures every cube rests flush on the floor.

Quick start
-----------
Change NUM_CUBES below and run::

    python g1_description/scene_generator.py

Or call from Python::

    from g1_description.scene_generator import generate_scene_xml
    xml = generate_scene_xml(num_cubes=50, seed=42)

Command-line options
--------------------
  --num-cubes N     Number of cubes (default: NUM_CUBES)
  --seed      S     Random seed for reproducibility
  --output    PATH  Output XML path (default: scene_withcamera.xml)
"""
import os
import random
import argparse

# ─── Quick control variable ───────────────────────────────────────────────────
NUM_CUBES: int = 200    # ← change this to generate a different number of cubes
# ─────────────────────────────────────────────────────────────────────────────

_SCENE_HEADER = """\
<mujoco model="g1 scene">
  <include file="g1_29dof_rev_1_0.xml"/>

  <statistic center="1.0 0.7 1.0" extent="0.8"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.1 0.1 0.1" specular="0.9 0.9 0.9"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-140" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="flat" rgb1="0 0 0" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
      rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true"
      texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    <camera name="rgb_camera" pos="0 0 1.5" euler="0 0 0" fovy="60"/>
    <body name="cube" pos="0.75 0 0.05">
      <freejoint/>
      <geom type="box" size="0.05 0.02 0.06" rgba="1 0 0 1" mass="0.01" friction="3.0 2.0 0.003"/>
    </body>
"""

_SCENE_FOOTER = """\
  </worldbody>
</mujoco>
"""


def generate_scene_xml(num_cubes: int = NUM_CUBES, seed: int = None) -> str:
    """Return a MuJoCo XML string with *num_cubes* randomly placed boxes.

    Parameters
    ----------
    num_cubes : int
        How many cubes to generate.
    seed : int, optional
        Random seed for reproducible layouts.
    """
    rng = random.Random(seed)

    lines = [
        f"    <!-- {num_cubes} auto-generated cubes  "
        f"(single geom → collision == render geometry) -->"
    ]
    for i in range(num_cubes):
        sx   = rng.uniform(0.015, 0.070)   # half-extent x
        sy   = rng.uniform(0.015, 0.070)   # half-extent y
        sz   = rng.uniform(0.020, 0.080)   # half-extent z
        px   = rng.uniform(0.30,  1.80)    # world x
        py   = rng.uniform(-1.00, 1.00)    # world y
        pz   = sz                           # bottom face on floor
        mass = rng.uniform(0.010, 0.040)
        r, g, b = rng.random(), rng.random(), rng.random()

        lines.append(
            f'    <body name="cube_gen_{i:03d}" pos="{px:.3f} {py:.3f} {pz:.3f}">\n'
            f'      <freejoint/>\n'
            f'      <geom name="cube_gen_{i:03d}_geom" type="box"'
            f' size="{sx:.3f} {sy:.3f} {sz:.3f}"'
            f' rgba="{r:.2f} {g:.2f} {b:.2f} 1"'
            f' mass="{mass:.3f}" friction="3.0 2.0 0.003"/>\n'
            f'    </body>'
        )

    return _SCENE_HEADER + "\n".join(lines) + "\n" + _SCENE_FOOTER


def write_scene(num_cubes: int = NUM_CUBES, seed: int = None,
                output: str = None) -> str:
    """Generate and write the scene XML.  Returns the output path."""
    if output is None:
        output = os.path.join(os.path.dirname(__file__), "scene_withcamera.xml")
    xml = generate_scene_xml(num_cubes, seed)
    with open(output, "w", encoding="utf-8") as f:
        f.write(xml)
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate MuJoCo scene XML.")
    parser.add_argument("--num-cubes", type=int, default=NUM_CUBES,
                        help=f"Number of cubes to generate (default: {NUM_CUBES})")
    parser.add_argument("--seed", type=int, default=None,
                        help="Random seed for reproducible layout")
    parser.add_argument("--output", default=None,
                        help="Output XML path (default: scene_withcamera.xml)")
    args = parser.parse_args()

    out = write_scene(args.num_cubes, args.seed, args.output)
    print(f"[scene_generator] {args.num_cubes} cubes → {out}")
