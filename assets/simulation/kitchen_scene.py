"""
Minimal Isaac Sim 6.0 kitchen scene.

Run headlessly:
    ./python.sh assets/simulation/kitchen_scene.py

The scene loads the authored static environment (assets/scene/kitchen_scene.usd
— light, floor, cooking table, robot support and the Franka Panda arm) and then
references the movable ingredients: the hamburger on the left of the table and
the case on the right.  The isaacsim.ros2.bridge extension is enabled so the
scene subscribes to /kinematic_kitchen/prepare_order (std_msgs/String, payload:
order ID).  On each message the arm executes a simple reach-and-return joint
motion representing order preparation.
"""

from pathlib import Path

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": False})

# All Omniverse imports must come after SimulationApp is instantiated.
import numpy as np
import omni.usd
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
from pxr import Gf, Usd, UsdGeom, UsdPhysics

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ---------------------------------------------------------------------------------
# Enable ROS 2 bridge extension
# ---------------------------------------------------------------------------------

app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# ---------------------------------------------------------------------------------
# Scene setup — load the authored static environment, then the ingredients
# ---------------------------------------------------------------------------------

SCENE_DIR = Path(__file__).resolve().parent.parent / "scene"
KITCHEN_SCENE_USD = str(SCENE_DIR / "KitchenScene.usd")
HAMBURGER_USD = str(SCENE_DIR / "hamburguer" / "Hamburguer.usd")
CASE_USD = str(SCENE_DIR / "hamburguer" / "Case.usd")

# Prop placement on the table (metres, Z-up) — TUNE to your table layout.
# These assets are authored in non-metre units, so they need scaling down.
# Adjust each scale until the prop is real-world sized (a burger is ~0.12 m).
HAMBURGER_POSITION = (0.5, 0.4, 0.7)  # left side of the table
CASE_POSITION = (0.5, -0.4, 0.73)  # right side of the table
CASE_ORIENTATION = (0, 0, 180)

def add_prop(
    prim_path: str,
    usd_path: str,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float] = (0, 0, 0),
    dynamic: bool = False,
) -> None:
    """Reference a prop under an Xform we own, place it, and optionally make it
    a dynamic rigid body that falls under gravity and collides."""
    xform = UsdGeom.Xform.Define(stage, prim_path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*orientation))
    xform.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))
    stage_utils.add_reference_to_stage(usd_path=usd_path, path=f"{prim_path}/Model")

    if dynamic:
        body = stage.GetPrimAtPath(prim_path)
        UsdPhysics.RigidBodyAPI.Apply(body)
        # Dynamic bodies require a convex collision approximation, not a trimesh.
        for mesh in Usd.PrimRange(body):
            if mesh.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(mesh)
                UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr(
                    "convexHull"
                )


stage_utils.open_stage(KITCHEN_SCENE_USD)
stage = omni.usd.get_context().get_stage()

add_prop("/World/Hamburger", HAMBURGER_USD, HAMBURGER_POSITION, dynamic=True)
add_prop("/World/Case", CASE_USD, CASE_POSITION, CASE_ORIENTATION, dynamic=True)

# Franka prim path as authored in kitchen_scene.usd.
franka = Articulation("/World/franka")

app_utils.play()
simulation_app.update()

# ---------------------------------------------------------------------------------
# ROS 2 preparation subscriber
# ---------------------------------------------------------------------------------

_pending_orders: list[str] = []


class PrepareOrderSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("kinematic_kitchen_scene")
        self.create_subscription(
            String,
            "/kinematic_kitchen/prepare_order",
            self._on_prepare_order,
            10,
        )

    def _on_prepare_order(self, msg: String) -> None:
        self.get_logger().info(f"Preparing order: {msg.data}")
        _pending_orders.append(msg.data)


rclpy.init()
subscriber = PrepareOrderSubscriber()

# ---------------------------------------------------------------------------------
# Joint motion helpers
# ---------------------------------------------------------------------------------

# Franka home configuration (7 arm joints + 2 finger joints = 9 DOF)
_HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04])
_REACH = np.array([0.0, 0.2, 0.0, -1.6, 0.0, 1.8, 0.785, 0.04, 0.04])

_REACH_STEPS = 60
_RETURN_STEPS = 60
_TOTAL_STEPS = _REACH_STEPS + _RETURN_STEPS


def _lerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    return a + (b - a) * t


def _target_for_frame(frame: int) -> np.ndarray:
    if frame < _REACH_STEPS:
        return _lerp(_HOME, _REACH, frame / _REACH_STEPS)
    return _lerp(_REACH, _HOME, (frame - _REACH_STEPS) / _RETURN_STEPS)


# ---------------------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------------------

_motion_frame = 0
_active_order: str | None = None

while simulation_app.is_running():
    rclpy.spin_once(subscriber, timeout_sec=0.0)
    simulation_app.update()

    if _active_order is None and _pending_orders:
        _active_order = _pending_orders.pop(0)
        _motion_frame = 0
        subscriber.get_logger().info(f"Starting motion for order {_active_order}")

    if _active_order is not None:
        franka.set_dof_position_targets(_target_for_frame(_motion_frame))
        _motion_frame += 1
        if _motion_frame >= _TOTAL_STEPS:
            subscriber.get_logger().info(f"Order {_active_order} preparation complete")
            _active_order = None

subscriber.destroy_node()
rclpy.shutdown()
simulation_app.close()
