from abc import ABC, abstractmethod
from enum import Enum
from typing import List
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
    def __init__(self, articulation: Articulation, cumotion_robot: CumotionRobot, world_binding: WorldBinding, arm_tolerance: float):
        super().__init__()
        self._articulation = articulation
        self._cumotion_robot = cumotion_robot
        self._arm_tolerance = arm_tolerance
        self._world_binding = world_binding
        
    @abstractmethod
    def generate_trajectory(self, target_position_array: List[float], current_time: float) -> Trajectory:
        pass

    def _current_arm_configuration(self):
        arm_indices = [self._articulation.dof_names.index(j) for j in self._cumotion_robot.controlled_joint_names]
        return self._articulation.get_dof_positions().numpy().flatten()[arm_indices]

import numpy as np

class LinearTrajectoryGenerator(SimplifiedTrajectoryGenerator):
    def __init__(self, articulation, cumotion_robot, world_binding, arm_tolerance):
        super().__init__(articulation, cumotion_robot, world_binding, arm_tolerance)
        self._trajectory_generator = TrajectoryGenerator(cumotion_robot, articulation.dof_names)

    def generate_trajectory(self, target_position_array: List[float], current_time: float) -> Trajectory:
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
    def __init__(self, articulation, cumotion_robot, world_binding, arm_tolerance):
        super().__init__(articulation, cumotion_robot, world_binding, arm_tolerance)

        self._planner = GraphBasedMotionPlanner(
            cumotion_robot=self._cumotion_robot,
            cumotion_world_interface=self._world_binding.get_world_interface()
        )
        self._mode: TargetSpace = TargetSpace.CSPACE

    def generate_trajectory(self, target_position_array: List[float], current_time: float) -> Trajectory:
        # Already at the target: nothing to plan or follow (the graph planner
        # finds no path for a zero-length problem). Happens whenever a cycle
        # starts with the arm resting at home.
        target = np.array(target_position_array)

        if np.max(np.abs(self._current_arm_configuration() - target)) < self._arm_tolerance:
            return True
        
        self._world_binding.synchronize_transforms()
        path = self._generate_path(target)

        return self._create_trajectory_for(path, current_time)

    def set_mode(self, mode: TargetSpace):
        self._mode = mode

    def _generate_path(self, target: np.ndarray):
        if (self._mode == TargetSpace.CSPACE):
            return self._planner.plan_to_cspace_target(self._current_arm_configuration(), target)
        
        elif (self._mode == TargetSpace.JOINTSPACE):
            return self._planner.plan_to_pose_target(self._current_arm_configuration(), target[0:3], target[4:])