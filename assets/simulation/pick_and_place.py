from assets.simulation.robot_arm import RobotArm
import numpy as np
from enum import Enum

# A Cartesian waypoint: world-frame position [x, y, z] and orientation
# quaternion [w, x, y, z].
Pose = tuple[np.ndarray, np.ndarray]

class States(Enum):
    IDLE = 0
    DRIVING_HOME_TO_START = 1
    DRIVING_TO_PRE_PICK = 2
    DRIVING_TO_PICK = 3
    CLOSING = 4
    DRIVING_BACK_PRE_PICK = 5
    DRIVING_PLACE = 6
    OPENING = 7
    DRIVING_HOME_TO_FINISH = 8

class PickAndPlace:
    """One pick-and-place cycle driven by a RobotArm, starting and ending at home.

    home is a joint configuration (c-space, one value per arm joint); the pick and place
    waypoints are Cartesian end-effector poses in the world frame. Call start()
    to arm one cycle; update() advances the state machine whenever the arm
    finishes its current motion.
    """

    def __init__(self, arm: RobotArm, home: np.ndarray, pre_pick: Pose, pick: Pose, place: Pose):
        self._arm = arm
        self._home = home
        self._pre_pick = pre_pick
        self._pick = pick
        self._place = place
        self._state = States.IDLE
        self._start_requested = False

    def start(self):
        self._start_requested = True

    def is_idle(self):
        return self._state == States.IDLE and not self._start_requested

    def update(self, current_time: float):
        if not self._arm.is_active():
            self.update_state(current_time)

        self._arm.update(current_time)

    def update_state(self, current_time: float):
        if self._state == States.IDLE:
            if self._start_requested:
                self._start_requested = False
                self.home_to_start(current_time)
        elif self._state == States.DRIVING_HOME_TO_START:
            self.pre_pick(current_time)
        elif self._state == States.DRIVING_TO_PRE_PICK:
            self.pick(current_time)
        elif self._state == States.DRIVING_TO_PICK:
            self.close()
        elif self._state == States.CLOSING:
            self.back_pre_pick(current_time)
        elif self._state == States.DRIVING_BACK_PRE_PICK:
            self.place(current_time)
        elif self._state == States.DRIVING_PLACE:
            self.open()
        elif self._state == States.OPENING:
            self.home_to_finish(current_time)
        elif self._state == States.DRIVING_HOME_TO_FINISH:
            self._state = States.IDLE

    def home_to_start(self, current_time: float):
        self._drive_home(States.DRIVING_HOME_TO_START, current_time)

    def pre_pick(self, current_time: float):
        self._arm.open()
        self._drive_to(self._pre_pick, States.DRIVING_TO_PRE_PICK, current_time)

    def pick(self, current_time: float):
        self._drive_to(self._pick, States.DRIVING_TO_PICK, current_time)

    def close(self):
        self._arm.close()
        self._state = States.CLOSING

    def back_pre_pick(self, current_time: float):
        self._drive_to(self._pre_pick, States.DRIVING_BACK_PRE_PICK, current_time)

    def place(self, current_time: float):
        self._drive_to(self._place, States.DRIVING_PLACE, current_time)

    def open(self):
        self._arm.open()
        self._state = States.OPENING

    def home_to_finish(self, current_time: float):
        self._drive_home(States.DRIVING_HOME_TO_FINISH, current_time)

    def _drive_home(self, next_state: States, current_time: float):
        self._transition(self._arm.set_cspace_target(self._home, current_time), next_state)

    def _drive_to(self, target: Pose, next_state: States, current_time: float):
        position, orientation = target
        self._transition(self._arm.set_pose_target(position, orientation, current_time), next_state)

    def _transition(self, plan_succeeded: bool, next_state: States):
        if plan_succeeded:
            self._state = next_state
        else:
            print(f"Pick and place aborted in {self._state.name}: no collision-free path")
            self._state = States.IDLE
