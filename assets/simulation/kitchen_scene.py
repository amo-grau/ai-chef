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
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.experimental.objects import Mesh
from assets.simulation.stage_utils import add_prop
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot_motion.experimental.motion_generation import (
    SceneQuery,
    TrackableApi,
    ObstacleStrategy,
    ObstacleConfiguration,
    ObstacleRepresentation,
    WorldBinding,
    TrajectoryFollower,
    JointState,
    RobotState
)
from isaacsim.robot_motion.cumotion import (
    CumotionRobot,
    CumotionWorldInterface,
    GraphBasedMotionPlanner
)

from isaacsim.robot_motion.cumotion import load_cumotion_supported_robot

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
# cuMotion's Franka c-space covers only the 7 arm joints (see robot.xrdf).
_REACH = np.array([0.0, 0.2, 0.0, -1.6, 0.0, 1.8, 0.785])

_REACH_STEPS = 60
_RETURN_STEPS = 60
_TOTAL_STEPS = _REACH_STEPS + _RETURN_STEPS


stage_utils.open_stage(KITCHEN_SCENE_USD)

add_prop("/World/Hamburger", HAMBURGER_USD, HAMBURGER_POSITION, dynamic=True)
add_prop("/World/Case", CASE_USD, CASE_POSITION, CASE_ORIENTATION, dynamic=True)

articulation = Articulation("/World/franka")

cumotion_robot = load_cumotion_supported_robot("franka")

scene_query = SceneQuery()
objects = scene_query.get_prims_in_aabb(
    search_box_origin=[0.0, 0.0, 0.0],
    search_box_minimum=[-100.0, -100.0, -100.0],
    search_box_maximum=[100.0, 100.0, 100.0],
    tracked_api=TrackableApi.PHYSICS_COLLISION,
    # The robot must not be a world obstacle for its own planner: cuMotion
    # handles self-collision via its robot model, and the Franka asset's
    # finger geometry carries non-unity scaling that world tracking rejects.
    exclude_prim_paths="/World/franka",
)
print("Scene Queried objects: ", objects)

# WorldBinding queries local scales/poses on tracked prims and requires the
# standard translate/orient/scale op stack; this rewrite preserves world poses.
XformPrim(paths=objects, reset_xform_op_properties=True)

obstacle_strategy = ObstacleStrategy()
obstacle_strategy.set_default_safety_tolerance(0.06)
# CumotionWorldInterface does not implement raw-mesh obstacles (add_meshes), so
# represent Mesh prims as oriented bounding boxes for planning.
obstacle_strategy.set_default_configuration(Mesh, ObstacleConfiguration("obb", 0.01))

world_interface = CumotionWorldInterface(visualize_debug_prims=True)
world_binding = WorldBinding(
    world_interface=world_interface,
    obstacle_strategy=obstacle_strategy,
    tracked_prims=objects,
    tracked_collision_api=TrackableApi.PHYSICS_COLLISION
)

world_binding.initialize()
world_binding.get_world_interface().update_world_to_robot_root_transforms(articulation.get_world_poses())

planner = GraphBasedMotionPlanner(
    cumotion_robot=cumotion_robot,
    cumotion_world_interface=world_binding.get_world_interface()
)
max_velocities = np.array([2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5])  # rad/s
max_accelerations = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0])  # rad/s²


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
_active_order: str | None = None
follower = TrajectoryFollower()

while simulation_app.is_running():
    rclpy.spin_once(order_subscriber, timeout_sec=0.0)
    simulation_app.update()
    
    if _active_order is None and order_subscriber.has_next():
        _active_order = order_subscriber.next()
        order_subscriber.get_logger().info(f"Starting motion for order {_active_order}")
        world_binding.synchronize_transforms()
        arm_indices = [articulation.dof_names.index(j) for j in cumotion_robot.controlled_joint_names]
        q_initial = articulation.get_dof_positions().numpy().flatten()[arm_indices]
        path = planner.plan_to_cspace_target(q_initial, _REACH)
        if path is None:
            order_subscriber.get_logger().error(f"No collision-free path found for order {_active_order}")
            _active_order = None
            continue
        trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=max_velocities,
            max_accelerations=max_accelerations,
            robot_joint_space=articulation.dof_names,
            active_joints=cumotion_robot.controlled_joint_names,
        )
        follower.set_trajectory(trajectory)
        joint_state = JointState.from_name(
            robot_joint_space=articulation.dof_names,
            positions=(articulation.dof_names, articulation.get_dof_positions()),
            velocities=(articulation.dof_names, articulation.get_dof_velocities())
        )
        estimated_state = RobotState(joints=joint_state)
        follower.reset(estimated_state, None, SimulationManager.get_simulation_time())

    if _active_order is not None:

        desired_state = follower.forward(estimated_state, None, SimulationManager.get_simulation_time())
        while desired_state is not None:
            if desired_state.joints.positions is not None:
                articulation.set_dof_position_targets(
                    positions=desired_state.joints.positions,
                    dof_indices=desired_state.joints.position_indices
                )

            simulation_app.update()
            estimated_state = RobotState(
                joints = JointState.from_name(
                    robot_joint_space=articulation.dof_names,
                    positions=(articulation.dof_names, articulation.get_dof_positions()),
                    velocities=(articulation.dof_names, articulation.get_dof_velocities())
                )
            )

            desired_state = follower.forward(estimated_state, None, SimulationManager.get_simulation_time())
            rclpy.spin_once(order_subscriber, timeout_sec=0.0)
            simulation_app.update()
        _active_order = None

order_subscriber.destroy_node()
rclpy.shutdown()
simulation_app.close()
