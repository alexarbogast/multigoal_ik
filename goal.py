import numpy as np

class GoalContext:
    def __init__(self):
        self._secondary = False
        self._weight = 1.0

        self._goal_link_names = []
        self._goal_variable_indices = []

        self._problem_active_variables = []
        self._problem_tip_link_names = []

        self._active_variable_positions_ = []
        self._tip_link_frames = []

    @property
    def secondary(self):
        return self._secondary

    @secondary.setter
    def secondary(self, is_sec):
        self._secondary = is_sec

    @property 
    def weight(self):
        return self._weight

    @weight.setter
    def weight(self, w):
        self._weight = w
    
    def addLink(self, link_name):
        self._goal_link_names.append(link_name)

    def getProblemLinkFrame(self, i):
        return self._tip_link_frames(i)
        

class Goal:
    def __init__(self):
        self._weight = 1.0
        self._secondary = False

    @property
    def secondary(self):
        return self._secondary

    @secondary.setter
    def secondary(self, is_sec):
        self._secondary = is_sec

    @property
    def weight(self):
        return self._weight

    @weight.setter
    def weight(self, w):
        self._weight = w

    def describe(self, context):
        context.secondary = self._secondary
        context.weight = self._weight

    def evaluate(self):
        return 0