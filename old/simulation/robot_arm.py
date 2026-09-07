
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

from old.simulation.drive_command import TrajectoryFactory, DriveCommand

class RobotArm:
    def __init__(self, articulation: Articulation, trajectory_factory: TrajectoryFactory, gripper_prim_path: str, arm_tolerance: float):
        self._articulation = articulation
        self._suction = self._wrap_suction_gripper(gripper_prim_path)

        self._trajectory_factory = trajectory_factory
        self._trajectory_follower = TrajectoryFollower()
        self._trajectory: Trajectory = None

        self._arm_tolerance = arm_tolerance
        self._is_opening = False
        self._is_closing = False

    def is_active(self):
        return not self._arm_at_final_state() or self._is_closing or self._is_opening
  
    def set_target(self, command: DriveCommand, current_time: float) -> bool:
        """Plan and start following a trajectory for the command.

        Returns False if no trajectory could be planned, so the caller can fall
        back or abort the cycle.
        """
        if not self._trajectory_factory.requires_motion(command):
            # Already there: nothing to follow, and the arm counts as idle.
            self._trajectory = None
            return True

        trajectory = self._trajectory_factory.create(command)
        if trajectory is None:
            return False

        self._trajectory = trajectory
        self._trajectory_follower.set_trajectory(self._trajectory)
        self._trajectory_follower.reset(self._estimated_state(), None, current_time)
        return True
        
    def open(self):
        self._is_opening = True
        self._suction.apply_gripper_action([-1.0])

    def close(self):
        self._is_closing = True
        self._suction.apply_gripper_action([1.0])
            
    def update(self, current_time: float):
        if not self._arm_at_final_state():
            estimated_state = self._estimated_state()
            desired_state = self._get_desired_state(current_time, estimated_state)

            self._articulation.set_dof_position_targets(
                positions=desired_state.joints.positions,
                dof_indices=desired_state.joints.position_indices
            )

        if self._is_closing:
            self._is_closing = not self._is_closed()

        if self._is_opening:
            self._is_opening = not self._is_opened()

    def _get_desired_state(self, current_time, estimated_state):
        """The commanded state for this step.

        current_time is absolute simulation time and the follower was reset at
        the motion's start, so it is passed through unchanged -- duration is a
        length, not an instant, and comparing the two mixes up the clocks.
        Once the trajectory clock expires the follower returns nothing; the
        physical arm still lags the commanded path, so hold the final target
        until _arm_at_final_state reports it has converged.
        """
        desired_state = self._trajectory_follower.forward(estimated_state, None, current_time)
        if desired_state is None or desired_state.joints.positions is None:
            return self._trajectory.get_target_state(self._trajectory.duration)
        return desired_state

    def _estimated_state(self):
        joint_state = JointState.from_name(
            robot_joint_space=self._articulation.dof_names,
            positions=(self._articulation.dof_names, self._articulation.get_dof_positions()),
            velocities=(self._articulation.dof_names, self._articulation.get_dof_velocities())
        )

        return RobotState(joints=joint_state)
    
    def _arm_at_final_state(self) -> bool:
        if self._trajectory == None:
            return True
        
        final_state = self._trajectory.get_target_state(self._trajectory.duration)
        target = final_state.joints.positions.numpy().flatten()
        indices = final_state.joints.position_indices.numpy().flatten()
        current = self._articulation.get_dof_positions().numpy().flatten()[indices]
        return bool(np.max(np.abs(current - target)) < self._arm_tolerance)

    def _is_opened(self):
        return self._gripper_status() == GripperStatus.Open

    def _is_closed(self):
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
            # The asset authors 0.02; tightening it to 0.01 left less slack
            # than the arm's own settling tolerance, so the cup could stop
            # just short of the hamburger and never latch.
            max_grip_distance=[0.02],
            coaxial_force_limit=[500.0],
            shear_force_limit=[500.0],
            retry_interval=[2.0],
        )
        return gripper