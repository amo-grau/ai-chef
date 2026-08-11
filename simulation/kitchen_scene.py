"""
Minimal Isaac Sim 6.0 kitchen scene.

Run headlessly:
    ./python.sh simulation/kitchen_scene.py

The scene loads the authored static environment (assets/scene/KitchenSceneUr.usd
— light, floor, cooking table, robot support and the UR10 arm with its suction
cup, the asset's Long_Suction gripper variant) and then references the movable
ingredients: the hamburger on the left of the table and the case on the right.
The isaacsim.ros2.bridge extension is enabled so the scene subscribes to
/kinematic_kitchen/prepare_order (kinematic_kitchen_interfaces/PrepareOrder,
carrying the order ID and its items).  On each message the arm executes a
pick-and-place cycle representing order preparation.
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
from simulation.stage_utils import add_prop, add_static_block
from isaacsim.core.simulation_manager import SimulationManager

from isaacsim.robot_motion.cumotion import load_cumotion_supported_robot

import rclpy

from simulation.for_handling_requests.prepare_order_subscriber import RosMotionCommandSubscriber
from simulation.robot_arm import RobotArm
from simulation.pick_and_place import PickAndPlace
from isaacsim.core.experimental.prims import XformPrim

# ---------------------------------------------------------------------------------
# Enable ROS 2 bridge extension
# ---------------------------------------------------------------------------------
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# ---------------------------------------------------------------------------------
# Scene setup — load the authored static environment, then the ingredients
# ---------------------------------------------------------------------------------
SCENE_DIR = Path(__file__).resolve().parent.parent / "assets" / "scene"
KITCHEN_SCENE_USD = str(SCENE_DIR / "KitchenSceneUr.usd")
HAMBURGER_USD = str(SCENE_DIR / "hamburguer" / "Hamburguer.usd")
CASE_USD = str(SCENE_DIR / "hamburguer" / "SplitCase.usd")

# OBJECT  WORLD FRAMES AND DIMENSIONS
OBSERVING_CAMERA_POSITION = [3.5, 0.0, 1.5]
CAMERA_TARGET = [0.0, 0.0, 0.7]
TABLE_POSITION = (0.85, 0, 0)
ObjectsDropHeight = 0.75
ObjectsFrontDistanceToRobot = 0.5
ObjectsSideDistanceToRobot = 0.4
HAMBURGER_POSITION = (ObjectsFrontDistanceToRobot, ObjectsSideDistanceToRobot, ObjectsDropHeight)  # left side of the table
CASE_POSITION = (ObjectsFrontDistanceToRobot, -ObjectsSideDistanceToRobot, ObjectsDropHeight)  # right side of the table
CASE_ORIENTATION = (0, 0, 180)
CASE_SUPPORT_SIZE = (0.1, 0.07, 0.03)  # width, depth, height (m)
CASE_SUPPORT_POSITION = (ObjectsFrontDistanceToRobot, -(ObjectsSideDistanceToRobot + 0.1), ObjectsDropHeight - 0.05)

# ROBOT C-SPACE FRAMES
HOME = np.array([-1.57*2, -1.57, -1.57, -1.57, 1.57, 0.0])

# ROBOT TARGET-SPACE FRAMES
DOWNWARDS_ORIENTATION = np.array([0.0, 0.0, 1.0, 0.0])
HAMBUREGUER_THICKNESS = 0.01
TABLE_HEIGHT = 0.8
HAMBURGER_HEIGHT = TABLE_HEIGHT + HAMBUREGUER_THICKNESS
TCP_Z_OFF = 0.08
PRE_PICK_OFFSET = 0.1
HAMBURGUER_DROP_HEIGHT = 0.06
CASE_Y_OFFSET_FOR_HAMBURGER = 0.055

PRE_PICK = (np.array([HAMBURGER_POSITION[0], HAMBURGER_POSITION[1], HAMBURGER_HEIGHT + PRE_PICK_OFFSET + TCP_Z_OFF]), DOWNWARDS_ORIENTATION)
PICK = (np.array([PRE_PICK[0][0], PRE_PICK[0][1], PRE_PICK[0][2] - PRE_PICK_OFFSET]), DOWNWARDS_ORIENTATION)

PLACE = (np.array([CASE_POSITION[0], CASE_POSITION[1] + CASE_Y_OFFSET_FOR_HAMBURGER, TABLE_HEIGHT + PRE_PICK_OFFSET + HAMBURGUER_DROP_HEIGHT + TCP_Z_OFF]), DOWNWARDS_ORIENTATION)
DROP = (np.array([PLACE[0][0], PLACE[0][1], PLACE[0][2] - PRE_PICK_OFFSET]), DOWNWARDS_ORIENTATION)

# TIME IN SECONDS
DROP_DWELL_SECONDS = 1.0

# PIRIM PATHS
robot_name = "ur10"
robot_prim_path = f"/World/{robot_name}"
table_prim_path = "/World/CokingTable"
hamburger_prim_path = "/World/Hamburger"
case_prim_path = "/World/Case"
case_support_path = "/World/CaseSupport"
gripper_prim_path = f"{robot_prim_path}/ee_link/SurfaceGripper"

# ROBOT CONFIGURATION
robot_max_velocities = np.array([2.16, 2.16, 3.15, 3.2, 3.2, 3.2])
robot_max_accelerations = np.array([4.0, 4.0, 4.0, 4.0, 4.0, 4.0])
arm_tolerance = 0.02

stage_utils.open_stage(KITCHEN_SCENE_USD)

table = XformPrim(table_prim_path)
table.reset_xform_op_properties()
table.set_world_poses(positions=[TABLE_POSITION])

add_prop(hamburger_prim_path, HAMBURGER_USD, HAMBURGER_POSITION, dynamic=True)
add_prop(case_prim_path, CASE_USD, CASE_POSITION, CASE_ORIENTATION)
add_static_block(
    case_support_path,
    CASE_SUPPORT_SIZE,
    CASE_SUPPORT_POSITION,
    material_path=f"{table_prim_path}/Looks/Aluminum_Brushed",
)

articulation = Articulation(robot_prim_path)

cumotion_robot = load_cumotion_supported_robot(robot_name)

ViewportManager.set_camera_view(
    "/OmniverseKit_Persp",
    eye=OBSERVING_CAMERA_POSITION,
    target=CAMERA_TARGET
)

app_utils.play()
simulation_app.update()

articulation.set_dof_positions(HOME)
articulation.set_dof_position_targets(HOME)
simulation_app.update()

rclpy.init()
order_subscriber = RosMotionCommandSubscriber()
from drive_command import *

# ---------------------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------------------
trajectory_factory = TrajectoryFactory(articulation, cumotion_robot, robot_max_velocities, robot_max_accelerations, robot_prim_path)
robot_arm = RobotArm(articulation, trajectory_factory, gripper_prim_path, arm_tolerance)
pick_and_place = PickAndPlace(robot_arm, HOME, PRE_PICK, PICK, PLACE, DROP, drop_dwell=DROP_DWELL_SECONDS)

while simulation_app.is_running():
    rclpy.spin_once(order_subscriber, timeout_sec=0.0)
    simulation_app.update()

    if order_subscriber.has_active_order() and order_subscriber.has_next():
        order_subscriber.next()
        pick_and_place.start()

    pick_and_place.update(SimulationManager.get_simulation_time())

    if order_subscriber.has_active_order() is not None and pick_and_place.is_idle():
        order_subscriber.on_active_order_handeled()

order_subscriber.destroy_node()
rclpy.shutdown()
simulation_app.close()
