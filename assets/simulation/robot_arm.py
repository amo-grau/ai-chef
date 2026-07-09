# This module uses Omniverse imports, so it must only be imported after the
# entry-point script has instantiated SimulationApp.
import numpy as np
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.experimental.objects import Mesh
from isaacsim.robot_motion.experimental.motion_generation import (
    SceneQuery,
    TrackableApi,
    ObstacleStrategy,
    ObstacleConfiguration,
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


class RobotArm:
    def __init__(self, articulation: Articulation, cumotion_robot: CumotionRobot):
        self._articulation = articulation
        self._cumotion_robot = cumotion_robot
        obstacle_strategy = ObstacleStrategy()
        obstacle_strategy.set_default_safety_tolerance(0.06)
        obstacle_strategy.set_default_configuration(Mesh, ObstacleConfiguration("obb", 0.01))
        scene_query = SceneQuery()
        collision_objects = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-100.0, -100.0, -100.0],
            search_box_maximum=[100.0, 100.0, 100.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
            # The robot must not be a world obstacle for its own planner: cuMotion
            # handles self-collision via its robot model, and the Franka asset's
            # finger geometry carries non-unity scaling that world tracking rejects.
            exclude_prim_paths="/World/franka",
        )
        # WorldBinding queries local scales/poses on tracked prims and requires the
        # standard translate/orient/scale op stack; this rewrite preserves world poses.
        XformPrim(paths=collision_objects, reset_xform_op_properties=True)
        world_interface = CumotionWorldInterface(visualize_debug_prims=True)
        self._world_binding = WorldBinding(
            world_interface=world_interface,
            obstacle_strategy=obstacle_strategy,
            tracked_prims=collision_objects,
            tracked_collision_api=TrackableApi.PHYSICS_COLLISION
        )

        self._world_binding.initialize()
        self._world_binding.get_world_interface().update_world_to_robot_root_transforms(articulation.get_world_poses())

        self._planner = GraphBasedMotionPlanner(
            cumotion_robot=self._cumotion_robot,
            cumotion_world_interface=self._world_binding.get_world_interface()
        )
        self._max_velocities = np.array([2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5])  # rad/s
        self._max_accelerations = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0])  # rad/s²
        self._trajectory_follower = TrajectoryFollower()
        self._is_moving = False

    def set_target(self, target: np.ndarray, current_time: float):
        self._world_binding.synchronize_transforms()
        arm_indices = [self._articulation.dof_names.index(j) for j in self._cumotion_robot.controlled_joint_names]
        q_initial = self._articulation.get_dof_positions().numpy().flatten()[arm_indices]
        path = self._planner.plan_to_cspace_target(q_initial, target)
        
        if path is None:
            print(f"No collision-free path found")
            self._is_moving = False
        
        else:
            trajectory = path.to_minimal_time_joint_trajectory(
                max_velocities=self._max_velocities,
                max_accelerations=self._max_accelerations,
                robot_joint_space=self._articulation.dof_names,
                active_joints=self._cumotion_robot.controlled_joint_names,
            )

            self._trajectory_follower.set_trajectory(trajectory)
            joint_state = JointState.from_name(
                robot_joint_space=self._articulation.dof_names,
                positions=(self._articulation.dof_names, self._articulation.get_dof_positions()),
                velocities=(self._articulation.dof_names, self._articulation.get_dof_velocities())
            )
            estimated_state = RobotState(joints=joint_state)
            self._trajectory_follower.reset(estimated_state, None, current_time)
            self._is_moving = True
            
    def update(self, current_time: float):
        estimated_state = RobotState(
            joints = JointState.from_name(
                robot_joint_space=self._articulation.dof_names,
                positions=(self._articulation.dof_names, self._articulation.get_dof_positions()),
                velocities=(self._articulation.dof_names, self._articulation.get_dof_velocities())
            )
        )

        desired_state = self._trajectory_follower.forward(estimated_state, None, current_time)

        if desired_state is not None and desired_state.joints.positions is not None:
            self._articulation.set_dof_position_targets(
                positions=desired_state.joints.positions,
                dof_indices=desired_state.joints.position_indices
            )
            self._is_moving = True
        else:
            self._is_moving = False

    def is_moving(self):
        return self._is_moving