import numpy as np

from .goal import Goal, GoalContext

class LinkGoalBase(Goal):
    def __init__(self, link_name = "", weight = 1.0):
        self._weight = weight
        self._link_name = link_name

        super(LinkGoalBase, self).__init__()

    @property
    def link_name(self):
        return self._link_name

    @link_name.setter
    def link_name(self, name):
        self._link_name = name

    def describe(self, context):
        super(LinkGoalBase, self).describe(context)
        context.addLink(self._link_name)
    

class PositionGoal(LinkGoalBase):
    def __init__(self, position = np.zeros(3), link_name = "", weight = 1.0):
        self._position = position
        super(PositionGoal, self).__init__(link_name, weight)

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, pos):
        self._position = pos

    def evaluate(self, context):
        #TODO: evaulte position contraint
        return 0


if __name__ == "__main__":

    pose = np.array([5.0, 3.0, 2.0])
    goal = PositionGoal(position = pose, link_name = "link_name", weight = 5.0)

    context = GoalContext()

    goal.evaluate(context)