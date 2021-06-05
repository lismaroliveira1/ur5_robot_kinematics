
import numpy as np
from controller import Robot
from time import sleep

pi = np.pi


class Ur5Controller(Robot):
    def __init__(self, ur5):
        super(Ur5Controller, self).__init__()
        self.timeStep = 3
        self.ur5 = ur5

    def initUr5(self):
        # Instantiating the motors class for every UR5 joint
        self.elbow_joint = self.getDevice('elbow_joint')
        self.shoulder_lift_joint = self.getDevice('shoulder_lift_joint')
        self.shoulder_pan_joint = self.getDevice('shoulder_pan_joint')
        self.wrist_1_joint = self.getDevice('wrist_1_joint')
        self.wrist_2_joint = self.getDevice('wrist_2_joint')
        self.wrist_3_joint = self.getDevice('wrist_3_joint')

        # Instantiating the GPS class (This GPS are located in the grimper's UR5)
        self.gps = self.getDevice('gps')

        # Instantiating the PositionSensor class for every UR5 joint
        self.elbow_joint_sensor = self.getDevice('elbow_joint_sensor')
        self.shoulder_lift_joint_sensor = self.getDevice(
            'shoulder_lift_joint_sensor')
        self.shoulder_pan_joint_sensor = self.getDevice(
            'shoulder_pan_joint_sensor')
        self.wrist_1_joint_sensor = self.getDevice('wrist_1_joint_sensor')
        self.wrist_2_joint_sensor = self.getDevice('wrist_2_joint_sensor')
        self.wrist_3_joint_sensor = self.getDevice('wrist_3_joint_sensor')

        # Activating the sensors
        self.elbow_joint_sensor.enable(10)
        self.shoulder_lift_joint_sensor.enable(10)
        self.shoulder_pan_joint_sensor.enable(10)
        self.wrist_1_joint_sensor.enable(10)
        self.wrist_2_joint_sensor.enable(10)
        self.wrist_3_joint_sensor.enable(10)

        # Initial postion all theta joints is equal zero
        self.elbow_joint.setPosition(0)
        self.shoulder_lift_joint.setPosition(0)
        self.shoulder_pan_joint.setPosition(0)
        self.wrist_1_joint.setPosition(0)
        self.wrist_2_joint.setPosition(0)
        self.wrist_3_joint.setPosition(0)

    def forwardKinematic(self, theta1, theta2, theta3, theta4, theta5, theta6):
        self.dh = [
            {
                "alpha": 0,
                "a": 0,
                "distance": self.ur5.joint1.distance,
                "theta": theta1,
            },
            {
                "alpha": pi / 2,
                "a": 0,
                "distance": 0,
                "theta": theta2,
            },
            {
                "alpha": 0,
                "a": self.ur5.joint2.a,
                "distance": 0,
                "theta": theta3,
            },
            {
                "alpha": 0,
                "a": self.ur5.joint3.a,
                "distance": self.ur5.joint4.distance,
                "theta": theta4,
            },
            {
                "alpha": pi / 2,
                "a": 0,
                "distance": self.ur5.joint5.distance,
                "theta": theta5,
            },
            {
                "alpha": -pi / 2,
                "a": 0,
                "distance": self.ur5.joint6.distance,
                "theta": theta6,
            },
        ]
        tmatrix_list = []
        self.gps.enable(100)
        self.shoulder_pan_joint.setPosition(theta1)
        self.shoulder_lift_joint.setPosition(theta2)
        self.elbow_joint.setPosition(theta3)
        self.wrist_1_joint.setPosition(theta4)
        self.wrist_2_joint.setPosition(theta5)
        self.wrist_3_joint.setPosition(theta6)

        tMatrix0_6 = np.identity(4)
        for joint in self.dh:
            tMatrix = self.generateTMatrix(
                joint['theta'], joint['distance'], joint['a'], joint['alpha'])
            tMatrix0_6 = np.dot(tMatrix0_6, tMatrix)

    def inverseKinematic(self, point, rpy):
        x = point[0]
        y = point[1]
        z = point[2]

        r = 0.30
        a = (1 + y/x)
        b = (-2 * x - 2 * (y ** 2) / x)
        c = (x ** 2 + y ** 2 - r ** 2)

        delta = b ** 2 - 4 * a * c
        bestPoint_x = (-b - delta ** 1 / 2) / (2 * a)
        bestPoint_y = y / x * bestPoint_x
        bestPoint_z = z
        theta1 = np.arctan2(
            bestPoint_y, bestPoint_x) + np.arccos(self.dh[3]['distance'] / (x ** 2 + y ** 2) ** 1 / 2) + pi / 2
        if theta1 > pi:
            theta1 = np.arctan2(
                bestPoint_y, bestPoint_x) - np.arccos(self.dh[3]['distance'] / (x ** 2 + y ** 2) ** 1 / 2) + pi / 2
        self.shoulder_pan_joint.setPosition(theta1)
        theta5 = np.arccos((x * np.sin(theta1) -
                            y * np.cos(theta1) -
                            self.dh[3]['distance'])
                           / self.dh[5]['distance'])
        roll = rpy[0]
        pitch = rpy[1]
        yall = rpy[2]

        tMatrix = np.array([
            [np.cos(roll) * np.cos(yall) * np.cos(pitch) - np.sin(roll)*np.sin(yall),
             - np.cos(roll)*np.sin(yall)*np.cos(pitch) -
             np.sin(roll)*np.cos(yall),
             np.cos(roll)*np.sin(pitch), x],

            [np.sin(roll) * np.cos(yall) * np.cos(pitch) + np.cos(roll)*np.sin(yall),
             - np.sin(roll)*np.sin(yall)*np.cos(pitch) +
             np.cos(roll)*np.cos(yall),
             np.sin(roll)*np.sin(pitch), y],

            [-np.cos(roll) * np.sin(yall),
             np.sin(yall)*np.sin(pitch),
             np.cos(yall), z],
            [0, 0, 0, 1]
        ])

        xoY = tMatrix[0][1]
        yoY = tMatrix[1][1]
        xoX = tMatrix[0][0]
        yoX = tMatrix[1][0]
        p0_6 = np.dot(tMatrix, np.array(
            [[0], [0], [-self.ur5.joint6.distance], [1]]))
        theta6 = np.arctan2(
            (-xoY*np.sin(theta1) + yoY*np.cos(theta1)) / np.sin(theta5),
            (xoX*np.cos(theta1) - yoY*np.cos(theta1)) / np.sin(theta5))
        tMatrix0_1 = self.generateTMatrix(
            theta1, self.dh[0]['distance'], self.dh[0]['a'], self.dh[0]['alpha'])
        tMatrix4_5 = self.generateTMatrix(
            theta5, self.dh[4]['distance'], self.dh[4]['a'], self.dh[4]['alpha'])
        tMatrix5_6 = self.generateTMatrix(
            theta1, self.dh[5]['distance'], self.dh[5]['a'], self.dh[5]['alpha'])
        tMatrix0_5 = np.dot(tMatrix, np.linalg.inv(tMatrix5_6))
        tMatrix0_4 = np.dot(tMatrix0_5, np.linalg.inv(tMatrix4_5))
        tMatrix1_4 = np.dot(np.linalg.inv(tMatrix0_1), tMatrix0_4)
        p1_4XZ = tMatrix1_4[0][3] ** 2 + tMatrix1_4[2][3] ** 2
        a2 = self.ur5.joint2.a
        a3 = self.ur5.joint3.a
        theta3 = np.arccos((p1_4XZ - a2 ** 2 - a3 ** 2) / (2 * a2 * a3))
        theta2 = np.arctan2(
            tMatrix1_4[2][3], tMatrix1_4[0][3]) - np.arcsin(-a3*np.sin(theta3) / (p1_4XZ ** 1 / 2))
        tMatrix1_2 = self.generateTMatrix(
            theta2, self.dh[1]['distance'], self.dh[1]['a'], self.dh[1]['alpha'])
        tMatrix2_3 = self.generateTMatrix(
            theta3, self.dh[2]['distance'], self.dh[2]['a'], self.dh[2]['alpha'])
        tMatrix2_4 = np.dot(tMatrix1_2, np.linalg.inv(tMatrix1_4))
        tMatrix3_4 = np.dot(tMatrix2_3, np.linalg.inv(tMatrix2_4))
        theta4 = np.arctan2(tMatrix3_4[0][1], tMatrix3_4[0][0])

    def generateTMatrix(self, theta, distance, a, alpha):
        return np.array(
            [[np.cos(theta), -np.sin(0), 0, a, ],
             [np.sin(theta)*np.cos(alpha),
              np.cos(theta)*np.cos(alpha),
              - np.sin(alpha), -np.sin(alpha)*distance, ],
             [np.sin(theta)*np.sin(alpha),
             np.cos(theta)*np.sin(alpha),
             np.cos(alpha), np.cos(alpha)*distance, ],
             [0, 0, 0, 1], ])
