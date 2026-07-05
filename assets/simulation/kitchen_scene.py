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

import rclpy
from assets.simulation.prepare_order_subscriber import PrepareOrderSubscriber

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

# Franka home configuration (7 arm joints + 2 finger joints = 9 DOF)
_HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04])
_REACH = np.array([0.0, 0.2, 0.0, -1.6, 0.0, 1.8, 0.785, 0.04, 0.04])

_REACH_STEPS = 60
_RETURN_STEPS = 60
_TOTAL_STEPS = _REACH_STEPS + _RETURN_STEPS


stage_utils.open_stage(KITCHEN_SCENE_USD)

add_prop("/World/Hamburger", HAMBURGER_USD, HAMBURGER_POSITION, dynamic=True)
add_prop("/World/Case", CASE_USD, CASE_POSITION, CASE_ORIENTATION, dynamic=True)

franka = Articulation("/World/franka")

app_utils.play()
simulation_app.update()

rclpy.init()    
order_subscriber = PrepareOrderSubscriber()

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
    rclpy.spin_once(order_subscriber, timeout_sec=0.0)
    simulation_app.update()

    if _active_order is None and order_subscriber.has_next():
        _active_order = order_subscriber.next()
        _motion_frame = 0
        order_subscriber.get_logger().info(f"Starting motion for order {_active_order}")

    if _active_order is not None:
        franka.set_dof_position_targets(_target_for_frame(_motion_frame))
        _motion_frame += 1
        if _motion_frame >= _TOTAL_STEPS:
            order_subscriber.get_logger().info(f"Order {_active_order} preparation complete")
            _active_order = None

order_subscriber.destroy_node()
rclpy.shutdown()
simulation_app.close()
