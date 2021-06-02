import math
import numpy as np
from controller import Robot
from time import sleep


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
                "alpha": math.pi / 2,
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
                "alpha": math.pi / 2,
                "a": 0,
                "distance": self.ur5.joint5.distance,
                "theta": theta5,
            },
            {
                "alpha": -math.pi / 2,
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
            tMatrix = np.array([
                [np.cos(joint['theta']),
                 -np.sin(0),
                 0,
                 joint['a'], ],

                [np.sin(joint['theta'])*np.cos(joint['alpha']),
                 np.cos(joint['theta'])*np.cos(joint['alpha']),
                 - np.sin(joint['alpha']),
                 - np.sin(joint['alpha'])*joint['distance'], ],

                [math.sin(joint['theta'])*math.sin(joint['alpha']),
                 math.cos(joint['theta'])*math.sin(joint['alpha']),
                 math.cos(joint['alpha']),
                 math.cos(joint['alpha'])*joint['distance'], ],

                [0, 0, 0, 1],
            ]
            )
            """ print(tMatrix)
            tMatrix0_6 = np.dot(tMatrix0_6, tMatrix)
        print(tMatrix0_6)
        self.step(200)
        print(self.gps.getValues()) """

    def inverseKinematic(self, point):
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
        print([x, y, z])
        print([bestPoint_x, bestPoint_y, bestPoint_z])
        theta1 = np.arctan2(
            bestPoint_y, bestPoint_x) + np.arccos(self.dh[3]['distance'] / (x ** 2 + y ** 2) ** 1 / 2) + math.pi / 2
        print(theta1)
        """ print(theta1)
        print(self.dh[5]['distance'])
        print(np.arccos((xob * np.sin(theta1) -
                         yob * np.cos(theta1) -
                         self.dh[3]['distance'])
              / self.dh[5]['distance'])) """

    def run(self):
        while self.step(20) != 10:
            print('**')
