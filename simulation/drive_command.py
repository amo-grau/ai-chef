from abc import ABC, abstractmethod
from enum import Enum
from typing import List
import numpy as np
import cumotion

from isaacsim.robot_motion.experimental.motion_generation import Trajectory, WorldBinding
from isaacsim.robot_motion.cumotion import CumotionRobot, GraphBasedMotionPlanner, TrajectoryGenerator
from isaacsim.core.experimental.prims import Articulation
from isaacsim.robot_motion.cumotion.impl.utils import isaac_sim_to_cumotion_pose

class GripperAction(Enum):
    IDLE:0
    GRIP:1
    RELEASE:2

class TargetSpace(Enum):
    CSPACE: 0
    TASKSPACE: 1

class TrajectoryType(Enum):
    LINEAR: 0
    OPTIMIZED: 1

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
        arm_indices = [self._articulation.dof_names.index(j) for j in self._cumotion_robot.controlled_joint_names]
        return self._articulation.get_dof_positions().numpy().flatten()[arm_indices]

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
            orientation_world_to_target=target_position_array[4:],
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=orientation_world_to_base,
        )

        path_spec = cumotion.create_task_space_path_spec(initial_pose)
        path_spec.add_linear_path(target_pose)
        # Seed the IK on the measured configuration so the conversion stays on
        # the arm's current branch instead of jumping to another solution.
        ik_config = cumotion.IkConfig()
        ik_config.cspace_seeds = [current]
        return self._trajectory_generator.generate_trajectory_from_path_specification(
            path_spec, inverse_kinematics_config=ik_config
        )   

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
        
        elif (self._mode == TargetSpace.JOINTSPACE):
            return self._planner.plan_to_pose_target(self._current_arm_configuration(), target[0:3], target[4:])

class TrajectoryFactory:
    def __init__(self, articulation: Articulation, cumotion_robot: CumotionRobot, world_binding: WorldBinding, max_velocities, max_accelerations):
        self._articulation = articulation
        self._cumotion_robot = cumotion_robot
        self._world_binding = world_binding
        self._max_velocities = max_velocities
        self._max_accelerations = max_accelerations
        
    def create(self, command: DriveCommand) -> Trajectory:
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