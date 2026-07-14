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
            exclude_prim_paths=["/World/franka", "/World/Hamburger"]
        )
        # WorldBinding queries local scales/poses on tracked prims and requires the
        # standard translate/orient/scale op stack; this rewrite preserves world poses.
        XformPrim(paths=collision_objects, reset_xform_op_properties=True)
        world_interface = CumotionWorldInterface(visualize_debug_prims=False)
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
        # Franka joint limits, from the cuMotion franka config (robot.urdf /
        # robot.xrdf): the trajectory runs as fast as the hardware allows.
        self._max_velocities = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])  # rad/s
        self._max_accelerations = np.array([15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0])  # rad/s²
        self._trajectory_follower = TrajectoryFollower()
        self._is_moving = False
        # Final state of the current trajectory; held as the drive target after
        # the trajectory clock expires, until the joints physically converge.
        self._final_state = None
        self._ARM_TOLERANCE = 0.02  # rad, per joint
        # Each Franka finger joint travels 0..0.04 m.
        self._OPENED_POSE = 0.04
        self._CLOSED_POSE = 0.0
        self._GRIPPER_TOLERANCE = 0.005
        # Driving panda_finger_joint1 is enough: the Franka articulation moves
        # the second finger in tandem.
        self._finger_indices = [
            self._articulation.dof_names.index("panda_finger_joint1")
        ]
        self._is_opening = False
        self._is_closing = False

    def set_pose_target(self, position: np.ndarray, orientation: np.ndarray, current_time: float) -> bool:
        """Plan to a world-frame pose (task-space) and start following the trajectory.

        orientation is a [w, x, y, z] quaternion; it is fully constrained.
        Returns False if no collision-free path exists.
        """
        self._world_binding.synchronize_transforms()
        path = self._planner.plan_to_pose_target(self._current_arm_configuration(), position, orientation)
        return self._follow_path(path, current_time)

    def set_cspace_target(self, target: np.ndarray, current_time: float) -> bool:
        """Plan to a joint configuration (7 arm joints) and start following the trajectory.

        Returns False if no collision-free path exists.
        """
        self._world_binding.synchronize_transforms()
        path = self._planner.plan_to_cspace_target(self._current_arm_configuration(), target)
        return self._follow_path(path, current_time)

    def _current_arm_configuration(self) -> np.ndarray:
        arm_indices = [self._articulation.dof_names.index(j) for j in self._cumotion_robot.controlled_joint_names]
        return self._articulation.get_dof_positions().numpy().flatten()[arm_indices]

    def _follow_path(self, path, current_time: float) -> bool:
        if path is None:
            self._is_moving = False
            return False

        trajectory = path.to_minimal_time_joint_trajectory(
            max_velocities=self._max_velocities,
            max_accelerations=self._max_accelerations,
            robot_joint_space=self._articulation.dof_names,
            active_joints=self._cumotion_robot.controlled_joint_names,
        )

        self._trajectory_follower.set_trajectory(trajectory)
        self._final_state = trajectory.get_target_state(trajectory.duration)
        joint_state = JointState.from_name(
            robot_joint_space=self._articulation.dof_names,
            positions=(self._articulation.dof_names, self._articulation.get_dof_positions()),
            velocities=(self._articulation.dof_names, self._articulation.get_dof_velocities())
        )
        estimated_state = RobotState(joints=joint_state)
        self._trajectory_follower.reset(estimated_state, None, current_time)
        self._is_moving = True
        return True
            
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
        elif self._is_moving and self._final_state is not None:
            # The trajectory clock has expired, but the physical arm lags the
            # commanded trajectory. Hold the final target and only report the
            # motion finished once the joints have actually converged on it.
            self._articulation.set_dof_position_targets(
                positions=self._final_state.joints.positions,
                dof_indices=self._final_state.joints.position_indices
            )
            self._is_moving = not self._arm_at_final_state()
            if not self._is_moving:
                self._final_state = None
        else:
            self._is_moving = False

        if self._is_closing:
            self._is_closing = not self._is_closed()
        if self._is_opening:
            self._is_opening = not self._is_opened()

    def _arm_at_final_state(self) -> bool:
        target = self._final_state.joints.positions.numpy().flatten()
        indices = self._final_state.joints.position_indices.numpy().flatten()
        current = self._articulation.get_dof_positions().numpy().flatten()[indices]
        return bool(np.max(np.abs(current - target)) < self._ARM_TOLERANCE)

    def is_active(self):
        return self._is_moving or self._is_closing or self._is_opening
    
    def open(self):
        self._is_opening = True
        self._set_gripper(self._OPENED_POSE)

    def close(self):
        self._is_closing = True
        self._set_gripper(self._CLOSED_POSE)

    def _set_gripper(self, pos: float) -> None:
        self._articulation.set_dof_position_targets([pos], dof_indices=self._finger_indices)

    def _is_opened(self):
        return self._finger_position() > self._OPENED_POSE - self._GRIPPER_TOLERANCE

    def _is_closed(self):
        # When grasping, the fingers stall on the object before reaching the
        # closed pose, so "closed" means: no longer open and no longer moving.
        no_longer_open = self._finger_position() < self._OPENED_POSE - self._GRIPPER_TOLERANCE
        stalled = abs(self._finger_velocity()) < 1e-3
        return no_longer_open and stalled

    def _finger_position(self) -> float:
        return float(self._articulation.get_dof_positions().numpy().flatten()[self._finger_indices[0]])

    def _finger_velocity(self) -> float:
        return float(self._articulation.get_dof_velocities().numpy().flatten()[self._finger_indices[0]])