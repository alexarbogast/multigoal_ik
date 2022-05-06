import numpy as np

from multigoal_ik.problem import Problem
from multigoal_ik.solvers import IKParams

class MultiGoalIKSolver:
    def __init__(self, robot, ik_params=None):
        if ik_params is None:
            ik_params = IKParams()

        self.ikparams = ik_params
        self.robot = robot

        self.ikparams = ik_params
        self.state = np.zeros(self.robot.q.size)

    def solve(self, seed_state, goals):
        if not seed_state.size == self.state.size:
            raise ValueError(f"seed size: {seed_state.size} differs from state size: {self.state.size}")

        state = seed_state

        problem = Problem(self.robot, goals, self.ikparams)