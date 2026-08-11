# This module uses Omniverse imports, so it must only be imported after the
# entry-point script has instantiated SimulationApp.
import numpy as np
import isaacsim.core.experimental.utils.app as app_utils

app_utils.enable_extension("isaacsim.robot.surface_gripper")
from isaacsim.core.experimental.prims import Articulation
from isaacsim.robot.surface_gripper import GripperView
from isaacsim.robot.surface_gripper.bindings._surface_gripper import GripperStatus
from isaacsim.robot_motion.experimental.motion_generation import (
    TrajectoryFollower,
    JointState,
    RobotState,
    Trajectory
)

from drive_command import TrajectoryFactory, DriveCommand

class RobotArm:
    def __init__(self, articulation: Articulation, trajectory_factory: TrajectoryFactory, gripper_prim_path: str, arm_tolerance: float):
        self._articulation = articulation
        self._suction = self._wrap_suction_gripper(gripper_prim_path)

        self._trajectory_factory = trajectory_factory
        self._trajectory_follower = TrajectoryFollower()

        self._is_moving = False
        # Final state of the current trajectory; held as the drive target after
        # the trajectory clock expires, until the joints physically converge.
        self._final_state = None
        self._arm_tolerance = arm_tolerance
        self._is_opening = False
        self._is_closing = False          

    def set_target(self, command: DriveCommand, current_time: float):
        trajectory = self._trajectory_factory.create(command)
        
        if trajectory is None or not self._starts_at_current_configuration(trajectory):
            self._is_moving = False
            return False

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

    def _starts_at_current_configuration(self, trajectory: Trajectory) -> bool:
        # The IK conversion may reach the start pose on a different solution
        # branch; following such a trajectory would make the arm jump to it.
        start = trajectory.get_target_state(0.0)
        target = start.joints.positions.numpy().flatten()
        indices = start.joints.position_indices.numpy().flatten()
        current = self._articulation.get_dof_positions().numpy().flatten()[indices]
        return bool(np.max(np.abs(current - target)) < 0.1)
            
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
        return bool(np.max(np.abs(current - target)) < self._arm_tolerance)

    def is_active(self):
        return self._is_moving or self._is_closing or self._is_opening
    
    def open(self):
        self._is_opening = True
        self._suction.apply_gripper_action([-1.0])

    def close(self):
        self._is_closing = True
        self._suction.apply_gripper_action([1.0])

    def _is_opened(self):
        return self._gripper_status() == GripperStatus.Open

    def _is_closed(self):
        # Closed means the suction cup has actually attached an object; while
        # commanded shut with nothing in range the status stays "Closing".
        return self._gripper_status() == GripperStatus.Closed

    def _gripper_status(self) -> GripperStatus:
        return GripperStatus(self._suction.get_surface_gripper_status()[0])

    def _wrap_suction_gripper(self, gripper_prim_path: str) -> GripperView:
        """Wrap the surface-gripper suction cup authored in the robot asset.

        The surface-gripper plugin registers the gripper when the timeline
        starts playing, so the view only needs its prim path. The asset's
        properties are overridden to make attachment forgiving: objects
        within max_grip_distance of the cup get rigidly attached on
        close(), and while "Closing" the attach keeps being retried (e.g.
        commanded slightly before contact) instead of giving up after one
        attempt.
        """
        gripper = GripperView(paths=gripper_prim_path)
        gripper.set_surface_gripper_properties(
            max_grip_distance=[0.01],
            coaxial_force_limit=[500.0],
            shear_force_limit=[500.0],
            retry_interval=[2.0],
        )
        return gripper