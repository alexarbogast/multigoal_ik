
class RobotFK:
    '''
    RobotFK: helper class for finding problem link transformation 
    given a python robotics_toolbox robot model
    '''
    def __init__(self, rtb_robot):
        self.robot = rtb_robot
        self.n_vars = self.robot.q.size