import numpy as np
import math

from multigoal_ik.problem import Problem
from .ik_base import *

@IKFactory.register_solver
class IKGradient(IKBase):
    def __init__(self, robot_info, ik_params):
        self._solution = np.zeros(robot_info.n_vars)
        self._best_solution = self._solution
        self._gradient = self._solution

        super(IKGradient, self).__init__(robot_info, ik_params)

    def initialize(self, problem):
        super(IKGradient, self).initialize(problem)

        # TODO: start with random solution for threads > 1
        self._solution = problem.initial_guess
        self._best_solution = self._solution

    def step(self):
        # compute gradient direction
        temp = self._solution
        jd = 0.0001

        for ivar in self.problem.active_variables:
            temp[ivar] = self._solution[ivar] - jd
            p1 = self.computeFitness(temp)

            temp[ivar] = self._solution[ivar] + jd
            p3 = self.computeFitness(temp)

            temp[ivar] = self._solution[ivar]
            self._gradient[ivar] = p3 - p1

        # normalize gradient direction
        sum = 0.0001
        for ivar in self.problem.active_variables:
            sum += abs(self._gradient[ivar])
        f = 1 / sum * jd
        for ivar in self.problem.active_variables:
            self._gradient[ivar] *= f

        # initialize line search
        temp2 = self._solution - self._gradient
        p1 = self.computeFitness(temp2)

        temp2 = self._solution + self._gradient
        p3 = self.computeFitness(temp2)

        p2 = (p1 + p3) * 0.5

        # linear step size estimation
        cost_diff = (p3 - p1) * 0.5
        joint_diff = p2 / cost_diff

        if not math.isfinite(joint_diff):
            joint_diff = 0.0

        # move along gradient direction by estimated step size
        for ivar in self.problem.active_variables:
            temp[ivar] = self.robot_info.clip(
                self._solution[ivar] - self._gradient[ivar] * joint_diff, ivar)

        if (self.computeFitness(temp) < self.computeFitness(self._solution)):
            # solution imporved
            self._solution = temp

        if (self.computeFitness(self._solution) < self.computeFitness(self._best_solution)):
            self._best_solution = self._solution

    def getSolution(self):
        return self._best_solution