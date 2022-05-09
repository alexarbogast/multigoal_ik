from enum import Enum
import numpy as np

from multigoal_ik import GoalContext
from multigoal_ik.goal_types import OrientationGoal, PositionGoal

class Problem:
    def __init__(self, robot, goals, ik_params, initial_guess, timeout=None):
        self._robot = robot
        self._params = ik_params
        self.initial_guess = initial_guess
        self.timeout = timeout
        
        self.goals = []
        self.secondary_goals = []

        self.tip_link_names = []
        self.active_variables = [] # indices of joints


        def addActiveVariable(var_idx):
            if not var_idx in self.active_variables:
                self.active_variables.append(var_idx)

        def addTipLink(name):
            if not name in self.tip_link_names:
                self.tip_link_names.append(name)

        for goal in goals:
            goal_info = Problem.GoalInfo(goal)
            goal.describe(goal_info.goal_context)
        
            # verify goal link names are in robot model
            try:
                for link_name in goal_info.goal_context._goal_link_names:
                    link = self._robot.link_dict[link_name]
                    addTipLink(link.name)
            except KeyError as ke:
                print("Key not found in link dictionary: {}".format(ke))
                raise

            # add unseen variables to active variables list
            for var_idx in goal_info.goal_context._goal_variable_indices:
                addActiveVariable(var_idx)

            # TODO: add goal type and initialize frame
            if isinstance(goal_info.goal, PositionGoal):
                goal_info.goal_type = Problem.GoalType.Position
            elif isinstance(goal_info.goal, OrientationGoal):
                goal_info.goal_type = Problem.GoalType.Orientation

            if goal_info.goal_context.secondary:
                self.secondary_goals.append(goal_info)
            else: 
                self.goals.append(goal_info)            
            
        # update active variables from active subtree
        for name in self.tip_link_names:
            link_path, *__ = self._robot.get_path(end=name)
            for link in link_path:
                if link.isjoint:
                    addActiveVariable(link.jindex)

        self._initialize()

    def _initialize(self):
        for gg in zip(self.goals, self.secondary_goals):
            for g in gg:
                g.goal_context._problem_active_variables = self.active_varaibles
                g.goal_context._problem_tip_link_names = self.problem_tip_link_names
    
    def computeGoalFitness(self, goals, tip_frames, active_variable_positions, weighted=True):
        if not type(goals) is list:
            goals = [goals]

        sum = 0.0
        for goal_info in goals:
            goal_info.goal_context._tip_link_frames = tip_frames
            goal_info.goal_context._active_variable_positions_ = active_variable_positions

            weight = goal_info.weight_sq if weighted else 1
            sum += goal_info.goal.evaluate(goal_info.goal_context) * weight

        return sum

    def checkSolutionActiveVariables(self, tip_frames, active_variable_positions):
        # check if position, orientation, and pose goals are met
        for goal in self.goals:
            if goal.goal_type == Problem.GoalType.Position:
                print(self._params.dpos)
                if self._params.dpos is not None:
                    p_dist = self.computeGoalFitness(goal, tip_frames, active_variable_positions, weighted=False)
                    if not np.sqrt(p_dist) <= self._params.dpos: return False

            elif goal.goal_type == Problem.GoalType.Orientation:
                pass
            elif goal.goal_type == Problem.GoalType.Pose:
                pass
            else:
                pass

        return True
            
    class GoalType(Enum):
        Unknown = 0
        Position = 1
        Orientation = 2
        Pose = 3

    class GoalInfo:
        def __init__(self, goal):
            self.goal = goal
            self.goal_context = GoalContext()
            self.weight = goal.weight
            self.weight_sq = self.weight * self.weight
            self.goal_type = Problem.GoalType.Unknown