from sys import float_info

class IKParams:
    def __init__(self):
        # Problem parameters
        self.dpos, self.drot, self.dtwist = float_info.max, float_info.max, 1e-5 


class IKBase:
    def __init__(self, robot, ik_params):
        self.robot = robot
        self.params = ik_params

    def initialize(self, problem):
        self.problem = problem


# dynamically instantiate solvers registered with decorators
class IKFactory():
    subclasses = {}

    @classmethod
    def register_solver(cls, subclass):
        cls.subclasses[subclass.__name__] = subclass
        return subclass

    @classmethod
    def create(cls, solver_type, robot, ik_params):
        if solver_type not in cls.subclasses:
            raise ValueError(f"Bad solver type {solver_type}")

        return cls.subclasses[solver_type](robot, ik_params)