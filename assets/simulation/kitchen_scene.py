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
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
from assets.simulation.stage_utils import add_prop
from isaacsim.core.simulation_manager import SimulationManager

from isaacsim.robot_motion.cumotion import load_cumotion_supported_robot

import rclpy
from assets.simulation.prepare_order_subscriber import PrepareOrderSubscriber
from assets.simulation.robot_arm import RobotArm
from assets.simulation.pick_and_place import PickAndPlace

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
HAMBURGER_POSITION = (0.5, 0.4, 0.7)  # left side of the table
CASE_POSITION = (0.5, -0.4, 0.73)  # right side of the table
CASE_ORIENTATION = (0, 0, 180)

# Pick-and-place waypoints as cuMotion c-space targets: 7 arm joint positions
# (see robot.xrdf). APPROXIMATE values — joint1 aims the arm at the hamburger
# (atan2(0.4, 0.5) ≈ 0.675 rad) and the case (-0.675 rad); tune by jogging the
# arm in the GUI and reading articulation.get_dof_positions().
_HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])  # Franka ready pose
_PRE_PICK = np.array([0.675, 0.2, 0.0, -1.6, 0.0, 1.8, 0.785])   # above the hamburger
_PICK = np.array([0.675, 0.45, 0.0, -1.25, 0.0, 1.7, 0.785])     # down at the hamburger
_PLACE = np.array([-0.675, 0.45, 0.0, -1.25, 0.0, 1.7, 0.785])   # over the case


stage_utils.open_stage(KITCHEN_SCENE_USD)

add_prop("/World/Hamburger", HAMBURGER_USD, HAMBURGER_POSITION, dynamic=True)
add_prop("/World/Case", CASE_USD, CASE_POSITION, CASE_ORIENTATION, dynamic=True)

articulation = Articulation("/World/franka")

cumotion_robot = load_cumotion_supported_robot("franka")

app_utils.play()
simulation_app.update()

rclpy.init()    
order_subscriber = PrepareOrderSubscriber()

# ---------------------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------------------
_active_order: str | None = None
robot_arm = RobotArm(articulation, cumotion_robot)
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
