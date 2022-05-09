
class IKParams:
    def __init__(self):
        # IKParallel parameters
        self.solver_class_name = "IKGradient"
        
        # Problem parameters
        self.dpos, self.drot, self.dtwist = None, None, 1e-5 


class IKBase:
    def __init__(self, robot_info, ik_params):
        self.robot_info = robot_info
        self.params = ik_params

        self.problem = None

    def initialize(self, problem):
        self.problem = problem
        self.robot_info.initialize(problem.tip_link_names)

    def step(self):
        raise NotImplementedError()

    def getSolution(self):
        raise NotImplementedError()

    def extractActiveVariables(self, variable_positions):
        # TODO: implement this
        return variable_positions

    def checkSolution(self, variable_positions):
        self.robot_info.applyConfiguration(variable_positions)
        frame_dict = self.robot_info.active_frame_dict

        success = self.problem.checkSolutionActiveVariables(frame_dict, self.extractActiveVariables(variable_positions))
        return success

    def computeFitness(self, variable_positions):
        self.robot_info.applyConfiguration(variable_positions)
        frame_dict = self.robot_info.active_frame_dict

        fit = self.problem.computeGoalFitness(self.problem.goals, frame_dict, self.extractActiveVariables(variable_positions))
        return fit


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