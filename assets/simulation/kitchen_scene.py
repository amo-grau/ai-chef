"""
Minimal Isaac Sim 6.0 kitchen scene.

Run headlessly:
    ./python.sh assets/simulation/kitchen_scene.py

The scene loads the authored static environment (assets/scene/KitchenSceneUr.usd
— light, floor, cooking table, robot support and the UR10 arm with its suction
cup, the asset's Long_Suction gripper variant) and then references the movable
ingredients: the hamburger on the left of the table and the case on the right.
The isaacsim.ros2.bridge extension is enabled so the scene subscribes to
/kinematic_kitchen/prepare_order (std_msgs/String, payload: order ID).  On each
message the arm executes a pick-and-place cycle representing order preparation.
"""

from pathlib import Path

from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": False})

# All Omniverse imports must come after SimulationApp is instantiated.
import numpy as np
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.rendering_manager import ViewportManager
from assets.simulation.stage_utils import add_prop
from isaacsim.core.simulation_manager import SimulationManager

from isaacsim.robot_motion.cumotion import load_cumotion_supported_robot

import rclpy
from assets.simulation.prepare_order_subscriber import PrepareOrderSubscriber
from assets.simulation.robot_arm import RobotArm
from assets.simulation.pick_and_place import PickAndPlace
from isaacsim.core.experimental.prims import XformPrim

# ---------------------------------------------------------------------------------
# Enable ROS 2 bridge extension
# ---------------------------------------------------------------------------------
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# ---------------------------------------------------------------------------------
# Scene setup — load the authored static environment, then the ingredients
# ---------------------------------------------------------------------------------
SCENE_DIR = Path(__file__).resolve().parent.parent / "scene"
KITCHEN_SCENE_USD = str(SCENE_DIR / "KitchenSceneUr.usd")
HAMBURGER_USD = str(SCENE_DIR / "hamburguer" / "Hamburguer.usd")
CASE_USD = str(SCENE_DIR / "hamburguer" / "Case.usd")
HAMBURGER_POSITION = (0.5, 0.4, 0.7)  # left side of the table
CASE_POSITION = (0.5, -0.4, 0.73)  # right side of the table
CASE_ORIENTATION = (0, 0, 180)
TABLE_POSITION = (0.85, 0, 0)

_HOME = np.array([-1.57, -1.57, -1.57, -1.57, 1.57, 0.0])

# Pick-and-place waypoints as Cartesian poses of the planner's tool frame
# (tool0, per the ur10 robot.xrdf) in the world frame: (position [x, y, z],
# orientation quaternion [w, x, y, z]). The suction cup tip sits 0.22 m past
# tool0 along its +z, so with the tool pointing down each waypoint is the
# intended cup-tip height plus that offset. The cup-tip z heights are
# APPROXIMATE (table top at z ≈ 0.65) — tune against the live scene.
# Same tool-frame orientation the arm has at _HOME (verified via FK): the
# suction cup points straight down at every waypoint.
_DOWNWARDS = np.array([0.0, 1.0, 0.0, 0.0])

_CUP_LENGTH = 0.12

_PRE_PICK = (np.array([0.5, 0.4, 0.95 + _CUP_LENGTH]), _DOWNWARDS)   # above the hamburger
# Cup tip stops ~4 cm above the hamburger top (z ≈ 0.67), centred on it;
# the suction's max grip distance (1 cm) closes the remaining gap without
# the cup ever touching and shoving the hamburger.
_PICK = (np.array([0.5, 0.4, 0.75 + _CUP_LENGTH]), _DOWNWARDS)       # down at the hamburger
_PLACE = (np.array([0.5, -0.4, 0.95 + _CUP_LENGTH]), _DOWNWARDS)     # over the case


stage_utils.open_stage(KITCHEN_SCENE_USD)


table = XformPrim("/World/CokingTable")
table.reset_xform_op_properties()
table.set_world_poses(positions=[TABLE_POSITION])

add_prop("/World/Hamburger", HAMBURGER_USD, HAMBURGER_POSITION, dynamic=True)
add_prop("/World/Case", CASE_USD, CASE_POSITION, CASE_ORIENTATION, dynamic=True)

articulation = Articulation("/World/ur10")

cumotion_robot = load_cumotion_supported_robot("ur10")

# Front view: camera on the +x side looking along -x at the table and robot.
ViewportManager.set_camera_view(
    "/OmniverseKit_Persp",
    eye=[3.5, 0.0, 1.5],
    target=[0.0, 0.0, 0.7],
)

app_utils.play()
simulation_app.update()

# The ur10 asset spawns with all joints at zero — the arm stretched
# horizontally through the cooking table. Teleport it to the home
# configuration so the first plan starts from a collision-free state.
articulation.set_dof_positions(_HOME)
articulation.set_dof_position_targets(_HOME)
simulation_app.update()

rclpy.init()
order_subscriber = PrepareOrderSubscriber()

# ---------------------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------------------
_active_order: str | None = None
robot_prim_path = "/World/ur10"
# The suction cup ships in the ur10 asset's Long_Suction gripper variant;
# grasping is done by suction because the hamburger is wider than a parallel
# gripper's finger span (issue #31).
gripper_prim_path = "/World/ur10/ee_link/SurfaceGripper"
# Velocity limits from the cuMotion ur10 robot.urdf. Accelerations are kept
# well below the xrdf's 12 rad/s² planning limit: at full limit the position
# drives lag and overshoot at trajectory stops, which near the pick pose is
# enough to ram the cup into the hamburger.
robot_max_velocities = np.array([2.16, 2.16, 3.15, 3.2, 3.2, 3.2])
robot_max_accelerations = np.array([4.0, 4.0, 4.0, 4.0, 4.0, 4.0])
arm_tolerance = 0.02
robot_arm = RobotArm(articulation, cumotion_robot, robot_prim_path, gripper_prim_path, robot_max_velocities, robot_max_accelerations, arm_tolerance)
pick_and_place = PickAndPlace(robot_arm, _HOME, _PRE_PICK, _PICK, _PLACE)

while simulation_app.is_running():
    rclpy.spin_once(order_subscriber, timeout_sec=0.0)
    simulation_app.update()

    if _active_order is None and order_subscriber.has_next():
        _active_order = order_subscriber.next()
        order_subscriber.get_logger().info(f"Starting pick and place for order {_active_order}")
        pick_and_place.start()

    pick_and_place.update(SimulationManager.get_simulation_time())

    if _active_order is not None and pick_and_place.is_idle():
        order_subscriber.get_logger().info(f"Order {_active_order} completed")
        _active_order = None

order_subscriber.destroy_node()
rclpy.shutdown()
simulation_app.close()
