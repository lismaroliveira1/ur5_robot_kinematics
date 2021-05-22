class UR5e:
    def __init__(self, joint1, joint2, joint3, joint4, joint5, joint6):
        self.joint1 = joint1
        self.joint2 = joint2
        self.joint3 = joint3
        self.joint4 = joint4
        self.joint5 = joint5
        self.joint6 = joint6

    def ur5JointToList(self):
        joints = [self.joint1, self.joint2, self.joint3,
                  self.joint4, self.joint5, self.joint6]
        return joints
