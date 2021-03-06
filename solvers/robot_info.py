from spatialmath import SE3

class RobotInfo:
    '''
    RobotInfo: helper class for finding problem link transformation 
    given a python robotics_toolbox robot model
    '''
    def __init__(self, rtb_robot):
        self.robot = rtb_robot
        self.n_vars = self.robot.q.size

        # dictionary of {name: str, frame: SE3}
        self._active_frame_dict = {}

    @property
    def active_frame_dict(self):
        return self._active_frame_dict

    def initialize(self, active_frame_names):
        self._active_frame_dict = dict.fromkeys(active_frame_names, SE3())

    def applyConfiguration(self, q):
        for name in self.active_frame_dict:
            self._active_frame_dict[name] = SE3(self.robot.fkine(q, end = name, fast=True))

    # clip input joint variable (value: j, index: i) between joint limits
    def clip(self, j, i):
        def clamp(num, min_value, max_value):
            return max(min(num, max_value), min_value)
        
        limits = self.robot.qlim[:, i]
        return clamp(j, limits[0], limits[1])
        