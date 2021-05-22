import math
import numpy as np
from controller import Robot


class Ur5Controller(Robot):
    def __init__(self, ur5):
        super(Ur5Controller, self).__init__()
        self.timeStep = 3
        self.ur5 = ur5

    def initUr5(self):
        self.elbow_joint = self.getDevice('elbow_joint')
        self.shoulder_lift_joint = self.getDevice('shoulder_lift_joint')
        self.shoulder_pan_joint = self.getDevice('shoulder_pan_joint')
        self.wrist_1_joint = self.getDevice('wrist_1_joint')
        self.wrist_2_joint = self.getDevice('wrist_2_joint')
        self.wrist_3_joint = self.getDevice('wrist_3_joint')

        self.elbow_joint_sensor = self.getDevice('elbow_joint_sensor')
        self.shoulder_lift_joint_sensor = self.getDevice(
            'shoulder_lift_joint_sensor')
        self.shoulder_pan_joint_sensor = self.getDevice(
            'shoulder_pan_joint_sensor')
        self.wrist_1_joint_sensor = self.getDevice('wrist_1_joint_sensor')
        self.wrist_2_joint_sensor = self.getDevice('wrist_2_joint_sensor')
        self.wrist_3_joint_sensor = self.getDevice('wrist_3_joint_sensor')

        self.elbow_joint.setPosition(0)
        self.shoulder_lift_joint.setPosition(0)
        self.shoulder_pan_joint.setPosition(0)
        self.wrist_1_joint.setPosition(0)
        self.wrist_2_joint.setPosition(0)
        self.wrist_3_joint.setPosition(0)

        self.elbow_joint_sensor.enable(10)
        self.shoulder_lift_joint_sensor.enable(10)
        self.shoulder_pan_joint_sensor.enable(10)
        self.wrist_1_joint_sensor.enable(10)
        self.wrist_2_joint_sensor.enable(10)
        self.wrist_3_joint_sensor.enable(10)

    def forwardKinematic(self):
        ur5Joints = self.ur5.updateJointsPositions(
            self.shoulder_pan_joint_sensor.getValue(),
            self.shoulder_lift_joint_sensor.getValue(),
            self.elbow_joint_sensor.getValue(), 1, 1, 1)

        tMatrixlist = []

        for joint in ur5Joints:
            tMatrix = [
                [math.cos(joint.theta), -math.sin(joint.theta), 0, joint.a],
                [math.sin(joint.theta)*math.cos(joint.alpha), math.cos(joint.theta) *
                 math.cos(joint.alpha), -math.sin(joint.alpha), -math.sin(joint.alpha)*joint.distance],
                [math.sin(joint.theta)*math.sin(joint.alpha),
                 math.cos(joint.theta)*math.sin(joint.alpha), math.cos(joint.alpha), math.cos(joint.alpha)*joint.distance], [0, 0, 0, 1]
            ]
            tMatrixlist.append(np.array(tMatrix))

        self.tMatrix_0_1 = tMatrixlist[0]
        self.tMatrix_1_2 = tMatrixlist[1]
        self.tMatrix_2_3 = tMatrixlist[2]
        self.tMatrix_3_4 = tMatrixlist[3]
        self.tMatrix_4_5 = tMatrixlist[4]
        self.tMatrix_5_6 = tMatrixlist[5]

        self.tMatrix_0_6 = np.array(np.identity(4))
        for tMatrix in tMatrixlist:
            self.tMatrix_0_6 *= self.tMatrix_0_6.dot(tMatrix)

    def inverseKinematic(self):

        print(self.tMatrix_0_6.dot(
            [[0], [0], [-self.ur5.joint6.distance], [1]]))

    def run(self):
        while self.step(self.timeStep) != 10:
            print(self.wrist_1_joint_sensor.getValue())
        print('stoped')
