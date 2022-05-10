import numpy as np
from threading import Lock

from .ik_base import IKFactory

class IKParallel:
    def __init__(self, fk_model, ik_params):
        self.model = fk_model
        
        self.solver = IKFactory.create(
            ik_params.solver_class_name,
            self.model, 
            ik_params)

        self.problem = None

        # solution info
        self.best_fitness = float('inf')
        self.result = np.zeros(self.model.n_vars)
        self.iteration_count = 0
        self.finished = False
        self.success = False

        # solver info TODO: these become lists or ndarrays when adding multithreading 
        self.solver_success = False
        self.solver_solutions = self.result
        self.solver_fitness = 0.0

    def initialize(self, problem):
        self.problem = problem

    def _solverthread(self, i):
        self.solver.initialize(self.problem)

        # TODO: Add timeout condition
        # is this actually faster that single threading?
        i = 0
        while not self.finished:
            if self.finished: break

            # run solver for a few steps
            self.solver.step()
            self.iteration_count += 1
            for i in range(3):
                if not self.finished:
                    self.solver.step()

            if self.finished: break

            # get solution and check stop criterion
            result = self.solver.getSolution()
            success = self.solver.checkSolution(result)

            if success: self.finished = True
            self.solver_success = success
            self.solver_solutions = result
            self.solver_fitness = self.solver.computeFitness(result)

            if success: break
            

    def solve(self):
        self.best_fitness = float('inf')
        self.result = np.zeros(self.model.n_vars)
        self.iteration_count = 0
        self.finished = False
        self.success = False

        self._solverthread(0)

        self.result = self.solver_solutions
        self.best_fitness = self.solver_fitness
        self.success = self.solver_success

        return (self.result, self.best_fitness, self.success)