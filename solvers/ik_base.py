
class IKParams:
    def __init__(self):
        # IKParallel parameters
        self.solver_class_name = "IKGradient"
        
        # Problem parameters
        self.dpos, self.drot, self.dtwist = None, None, 1e-5 


class IKBase:
    def __init__(self, fk_model, ik_params):
        self.model = fk_model
        self.params = ik_params

        self.problem = None

    def initialize(self, problem):
        self.problem = problem

    def step(self):
        raise NotImplementedError()

    def getSolution(self):
        raise NotImplementedError()

    def checkSolution(self, variable_positions):
        self.model.applyConfiguration(variable_positions)
        frame_dict = self.model.active_frame_dict

        self.problem.checkSolutionActiveVariables(frame_dict, variable_positions)

        success = True
        fitness = 1
        return success, fitness


# dynamically instantiate solvers registered with decorators
class IKFactory():
    subclasses = {}

    @classmethod
    def register_solver(cls, subclass):
        cls.subclasses[subclass.__name__] = subclass
        return subclass

    @classmethod
    def create(cls, solver_type, fk_model, ik_params):
        if solver_type not in cls.subclasses:
            raise ValueError(f"Bad solver type {solver_type}")

        return cls.subclasses[solver_type](fk_model, ik_params)