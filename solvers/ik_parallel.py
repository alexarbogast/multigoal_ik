import numpy as np
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
        self.success = False

    def initialize(self, problem):
        self.problem = problem

    def _solverthread(self, i):
        self.solver.initialize(self.problem)

    def solve(self):
        self.best_fitness = float('inf')
        self.result = np.zeros(self.model.n_vars)
        self.iteration_count = 0
        self.success = False

        return 0

