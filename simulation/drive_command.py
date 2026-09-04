import cumotion
import numpy as np
from enum import Enum
from typing import List
from abc import ABC, abstractmethod

from isaacsim.core.experimental.objects import Mesh
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.robot_motion.experimental.motion_generation import Trajectory, WorldBinding
from isaacsim.robot_motion.experimental.motion_generation import (
    SceneQuery,
    TrackableApi,
    ObstacleStrategy,
    ObstacleConfiguration,
    WorldBinding
)

from isaacsim.robot_motion.cumotion import (
    CumotionRobot,
    GraphBasedMotionPlanner,
    TrajectoryGenerator,
    CumotionWorldInterface
)
from isaacsim.robot_motion.cumotion.impl.utils import isaac_sim_to_cumotion_pose


def arm_configuration(articulation: Articulation, cumotion_robot: CumotionRobot) -> np.ndarray:
    """The measured positions of the joints cuMotion controls, in its joint order."""
    arm_indices = [articulation.dof_names.index(j) for j in cumotion_robot.controlled_joint_names]
    return articulation.get_dof_positions().numpy().flatten()[arm_indices]


class GripperAction(Enum):
    IDLE = 0
    GRIP = 1
    RELEASE = 2

class TargetSpace(Enum):
    CSPACE = 0
    TASKSPACE = 1

class TrajectoryType(Enum):
    LINEAR = 0
    OPTIMIZED = 1

class DriveCommand:
    def __init__(
            self,
            target_position_array: List[float],
            target_position_space: TargetSpace,
            desired_trajectory: TrajectoryType
        ):

        self.target_position_array = target_position_array
        self.target_position_space = target_position_space
        self.desired_trajectory = desired_trajectory

class SimplifiedTrajectoryGenerator(ABC):
    def __init__(self, articulation: Articulation, cumotion_robot: CumotionRobot, world_binding: WorldBinding):
        super().__init__()
        self._articulation = articulation
        self._cumotion_robot = cumotion_robot
        self._world_binding = world_binding
        
    @abstractmethod
    def generate_trajectory(self, target_position_array: List[float]) -> Trajectory:
        pass

    def _current_arm_configuration(self):
        return arm_configuration(self._articulation, self._cumotion_robot)

    def _starts_at_current_configuration(self, trajectory: Trajectory) -> bool:
        # The IK conversion may reach the start pose on a different solution
        # branch; following such a trajectory would make the arm jump to it.
        start = trajectory.get_target_state(0.0)
        target = start.joints.positions.numpy().flatten()
        indices = start.joints.position_indices.numpy().flatten()
        current = self._articulation.get_dof_positions().numpy().flatten()[indices]
        return bool(np.max(np.abs(current - target)) < 0.1)

class LinearTrajectoryGenerator(SimplifiedTrajectoryGenerator):
    def __init__(self, articulation, cumotion_robot, world_binding, max_velocities, max_accelerations):
        super().__init__(articulation, cumotion_robot, world_binding)
        self._trajectory_generator = TrajectoryGenerator(cumotion_robot, articulation.dof_names)
        linear_speed_fraction = 0.25
        cspace_trajectory_generator = self._trajectory_generator.get_cspace_trajectory_generator()
        cspace_trajectory_generator.set_velocity_limits(linear_speed_fraction * max_velocities.astype(np.float64))
        cspace_trajectory_generator.set_acceleration_limits(linear_speed_fraction * max_accelerations.astype(np.float64))

    def generate_trajectory(self, target_position_array: List[float]) -> Trajectory:
        current = self._current_arm_configuration().astype(np.float64)
        tool_frame = self._cumotion_robot.robot_description.tool_frame_names()[0]
        initial_pose = self._cumotion_robot.kinematics.pose(current, tool_frame)

        position_world_to_base, orientation_world_to_base = (
            self._world_binding.get_world_interface().get_world_to_robot_base_transform()
        )
        target_pose = isaac_sim_to_cumotion_pose(
            position_world_to_target=target_position_array[0:3],
            orientation_world_to_target=target_position_array[3:],
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=orientation_world_to_base,
        )

        path_spec = cumotion.create_task_space_path_spec(initial_pose)
        path_spec.add_linear_path(target_pose)
        # Seed the IK on the measured configuration so the conversion stays on
        # the arm's current branch instead of jumping to another solution.
        ik_config = cumotion.IkConfig()
        ik_config.cspace_seeds = [current]
        trajectory = self._trajectory_generator.generate_trajectory_from_path_specification(
            path_spec, inverse_kinematics_config=ik_config
        )
        if trajectory is None or not self._starts_at_current_configuration(trajectory):
            return None
        return trajectory

class OptimizedTrajectoryGenerator(SimplifiedTrajectoryGenerator):
    def __init__(self, articulation, cumotion_robot, world_binding, max_velocities, max_accelerations):
        super().__init__(articulation, cumotion_robot, world_binding)
        self._max_velocities = max_velocities
        self._max_accelerations = max_accelerations

        self._planner = GraphBasedMotionPlanner(
            cumotion_robot=self._cumotion_robot,
            cumotion_world_interface=self._world_binding.get_world_interface()
        )
        self._mode: TargetSpace = TargetSpace.CSPACE

    def generate_trajectory(self, target_position_array: List[float]) -> Trajectory:
        target = np.array(target_position_array)
        
        self._world_binding.synchronize_transforms()
        path = self._generate_path(target)
        if path is None:
            return None

        return path.to_minimal_time_joint_trajectory(
            max_velocities=self._max_velocities,
            max_accelerations=self._max_accelerations,
            robot_joint_space=self._articulation.dof_names,
            active_joints=self._cumotion_robot.controlled_joint_names,
        )

    def set_mode(self, mode: TargetSpace):
        self._mode = mode

    def _generate_path(self, target: np.ndarray):
        if (self._mode == TargetSpace.CSPACE):
            return self._planner.plan_to_cspace_target(self._current_arm_configuration(), target)
        
        elif (self._mode == TargetSpace.TASKSPACE):
            return self._planner.plan_to_pose_target(self._current_arm_configuration(), target[0:3], target[3:])

class TrajectoryFactory:
    def __init__(self, articulation: Articulation, cumotion_robot: CumotionRobot, max_velocities, max_accelerations, robot_prim_path: str, arm_tolerance: float):
        self._articulation = articulation
        self._cumotion_robot = cumotion_robot
        self._max_velocities = max_velocities
        self._max_accelerations = max_accelerations
        self._arm_tolerance = arm_tolerance

        obstacle_strategy = ObstacleStrategy()
        obstacle_strategy.set_default_safety_tolerance(0.06)
        obstacle_strategy.set_default_configuration(Mesh, ObstacleConfiguration("obb", 0.01))
        scene_query = SceneQuery()
        collision_objects = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-100.0, -100.0, -100.0],
            search_box_maximum=[100.0, 100.0, 100.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
            # The robot must not be a world obstacle for its own planner:
            # cuMotion handles self-collision via its robot model. The
            # hamburger is excluded so the planner lets the cup touch it.
            exclude_prim_paths=[robot_prim_path, "/World/Hamburger"]
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

        
    def requires_motion(self, command: DriveCommand) -> bool:
        """Whether the arm has to move at all to satisfy the command.

        The graph planner finds no path for a zero-length problem, so a c-space
        command that the arm already satisfies -- every cycle starts with the
        arm resting at home -- has to be recognised before planning.
        """
        if command.target_position_space != TargetSpace.CSPACE:
            return True

        current = arm_configuration(self._articulation, self._cumotion_robot)
        target = np.array(command.target_position_array)
        return bool(np.max(np.abs(current - target)) >= self._arm_tolerance)

    def create(self, command: DriveCommand) -> Trajectory | None:
        """Plan a trajectory for the command, or None if no plan exists."""
        generator = self._create_generator(command)
        return generator.generate_trajectory(command.target_position_array)

    def _create_generator(self, command: DriveCommand) -> SimplifiedTrajectoryGenerator:
        if (command.desired_trajectory == TrajectoryType.OPTIMIZED):
            generator = OptimizedTrajectoryGenerator(self._articulation, self._cumotion_robot, self._world_binding, self._max_velocities, self._max_accelerations)
            generator.set_mode(command.target_position_space)
            return generator
        
        if (command.target_position_space == TargetSpace.TASKSPACE and command.desired_trajectory.LINEAR):
            return LinearTrajectoryGenerator(self._articulation, self._cumotion_robot, self._world_binding, self._max_velocities, self._max_accelerations)
        
        raise NotImplementedError("TrajectoryGenerator implemented for given command.")