from .ik_base import *

@IKFactory.register_solver
class IKGradient(IKBase):
    def __init__(self, robot, ik_params):
        super(IKGradient, self).__init__(robot, ik_params)

    def initialize(self, problem):
        self.problem = problem

    def step(self):
        print("Hey there's a step")

    def getSolution(self):
        print("Hey there's a solution")