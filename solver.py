import numpy as np

from .problem import Problem
from .solvers import IKParams
from .solvers.ik_parallel import IKParallel

from .solvers.robot_info import RobotInfo

class MultiGoalIKSolver:
    def __init__(self, robot, ik_params=None):
        if ik_params is None:
            ik_params = IKParams()

        self.ikparams = ik_params
        self.robot = robot
        self.fk_model = RobotInfo(self.robot)

        self.ikparams = ik_params
        self.state = np.zeros(self.fk_model.n_vars)

        self.ik = IKParallel(self.fk_model, self.ikparams)

    def solve(self, seed_state, goals):
        if not seed_state.size == self.state.size:
            raise ValueError(f"seed size: {seed_state.size} differs from state size: {self.state.size}")

        initial_guess = np.copy(seed_state)
        problem = Problem(self.robot, goals, self.ikparams, initial_guess)

        self.ik.initialize(problem)
        result, fitness, success = self.ik.solve()

        # TODO: wrap angles

        return result, fitness, success