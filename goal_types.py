import numpy as np
from spatialmath import Quaternion

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
        self._position = position # in model baseframe
        super(PositionGoal, self).__init__(link_name, weight)

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, pos):
        self._position = pos

    def evaluate(self, context):
        dist_vec = self._position - context.getProblemLinkFrame(self.link_name).t
        return np.square(dist_vec).sum()


class OrientationGoal(LinkGoalBase):
    def __init__(self, orientation = Quaternion(), link_name = "", weight = 1.0):
        self._orientation = orientation
        super(OrientationGoal, self).__init__(link_name, weight)

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, orientation):
        self._orientation = orientation

    def evaluate(self, context):
        #TODO: evaluate orientation constraint
        return 0


if __name__ == "__main__":

    pose = np.array([5.0, 3.0, 2.0])
    goal = PositionGoal(position = pose, link_name = "link_name", weight = 5.0)

    context = GoalContext()

    goal.evaluate(context)