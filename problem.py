from multigoal_ik import GoalContext


class Problem:
    def __init__(self, robot, goals, ik_params, timeout=None):
        self._params = ik_params
        self._robot = robot

        self._dpos = self._params.dpos
        self._drot = self._params.drot
        self._dtwist = self._params.dtwist
        
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
    
    def computeGoalFitness(self, goals, tip_frames, active_variable_positions):
        if not type(goals) is list:
            goals = [goals]

        sum = 0.0
        for goal_info in goals:
            goal_info.goal_context._tip_link_frames = tip_frames
            goal_info.goal_context._active_variable_positions_ = active_variable_positions
            sum += goal_info.goal.evaluate(goal_info.goal_context)

        return sum

    class GoalInfo:
        def __init__(self, goal):
            self.goal = goal
            self.goal_context = GoalContext()
            self.weight = goal.weight
            self.weight_sq = self.weight * self.weight