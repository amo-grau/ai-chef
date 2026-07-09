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

# cuMotion's Franka c-space covers only the 7 arm joints (see robot.xrdf).
_REACH = np.array([0.0, 0.2, 0.0, -1.6, 0.0, 1.8, 0.785])


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

while simulation_app.is_running():
    rclpy.spin_once(order_subscriber, timeout_sec=0.0)
    simulation_app.update()
    
    if _active_order is None and order_subscriber.has_next():
        _active_order = order_subscriber.next()
        order_subscriber.get_logger().info(f"Starting motion for order {_active_order}")
        robot_arm.set_target(_REACH, SimulationManager.get_simulation_time())

    if _active_order is not None:
        robot_arm.update(SimulationManager.get_simulation_time())

        if not robot_arm.is_moving():
            _active_order = None

order_subscriber.destroy_node()
rclpy.shutdown()
simulation_app.close()
