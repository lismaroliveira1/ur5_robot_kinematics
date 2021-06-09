
import numpy as np
from controller import Robot
from time import sleep

pi = np.pi


class Ur5Controller(Robot):
    def __init__(self, ur5):
        super(Ur5Controller, self).__init__()
        self.timeStep = 50
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
        self.step(100)
        self.elbow_joint.setPosition(0)
        self.shoulder_lift_joint.setPosition(-pi/2)
        self.shoulder_pan_joint.setPosition(0)
        self.wrist_1_joint.setPosition(0)
        self.wrist_2_joint.setPosition(0)
        self.wrist_3_joint.setPosition(0)

    def dhParams(self):
        dh = [
                {
                    "alpha": 0,
                    "a": 0,
                    "distance": self.ur5.joint1.distance,
                    "theta": self.ur5.joint1.theta,
                },
                {
                    "alpha": pi / 2,
                    "a": 0,
                    "distance": 0,
                    "theta": self.ur5.joint2.theta,
                },
                {
                    "alpha": 0,
                    "a": self.ur5.joint2.a,
                    "distance": 0,
                    "theta": self.ur5.joint3.theta,
                },
                {
                    "alpha": 0,
                    "a": self.ur5.joint3.a,
                    "distance": self.ur5.joint4.distance,
                    "theta": self.ur5.joint4.theta,
                },
                {
                    "alpha": pi / 2,
                    "a": 0,
                    "distance": self.ur5.joint5.distance,
                    "theta": self.ur5.joint5.theta,
                },
                {
                    "alpha": -pi / 2,
                    "a": 0,
                    "distance": self.ur5.joint6.distance,
                    "theta": self.ur5.joint6.theta,
                },
            ]
        return dh

    def generateTMatrix(self, theta, distance, a, alpha):
        return np.array(
            [[np.cos(theta), -np.sin(theta), 0, a ],
             [np.sin(theta)*np.cos(alpha),
              np.cos(theta)*np.cos(alpha),
              - np.sin(alpha), -np.sin(alpha)*distance ],
             [np.sin(theta)*np.sin(alpha),
             np.cos(theta)*np.sin(alpha),
             np.cos(alpha), np.cos(alpha)*distance ],
             [0, 0, 0, 1],])
             
    def forwardKinematic(self,angleJoint1, angleJoint2, angleJoint3, angleJoint4, angleJoint5, angleJoint6):
        self.ur5.updateJointsPositions(angleJoint1, angleJoint2, angleJoint3, angleJoint4, angleJoint5, angleJoint6)
        dh = self.dhParams()
        tMatrix0_6 = np.identity(4)
        for joint in dh:
            tMatrix = generateTMatrix(
                joint['theta'], joint['distance'], joint['a'], joint['alpha'])
            tMatrix0_6 = np.dot(tMatrix0_6, tMatrix)
        return tMatrix0_6

    def invKinematic(self,x, y, z):
        results = []
        dh = self.dhParams()
        rotation = np.array([[0, -1, 0],[0 , 0,-1],[-1, 0,0]])
        tMatrix = np.vstack((np.hstack((rotation, [[x],[y], [z]])), [0,0,0,1]))
        p0_5 = np.dot(tMatrix, np.array([[0], [0], [-self.ur5.joint6.distance], [1]]))
        p0_5x = p0_5[0][0]
        p0_5y = p0_5[1][0]
        theta1 = np.arctan2(p0_5y, p0_5x) + np.arccos(dh[3]['distance'] / ((p0_5x ** 2 + p0_5y ** 2) ** (1 / 2))) + pi / 2
        p0_6x = tMatrix[0][3]
        p0_6y = tMatrix[1][3]
        theta5  = np.arccos((p0_6x * np.sin(theta1) - p0_6y * np.cos(theta1) -  dh[3]['distance']) / dh[5]['distance'])
        xoY = tMatrix[0][1]
        yoY = tMatrix[1][1]
        xoX = tMatrix[0][0]
        yoX = tMatrix[1][0]
        theta6 = np.arctan2((-xoY*np.sin(theta1) + yoY*np.cos(theta1)) / np.sin(theta5),  (xoX*np.sin(theta1) - yoX*np.cos(theta1)) / np.sin(theta5))
        tMatrix0_1 = self.generateTMatrix(theta1, dh[0]['distance'], dh[0]['a'], dh[0]['alpha'])
        tMatrix4_5 = self.generateTMatrix(theta5, dh[4]['distance'], dh[4]['a'], dh[4]['alpha'])
        tMatrix5_6 = self.generateTMatrix(theta6, dh[5]['distance'], dh[5]['a'], dh[5]['alpha'])
        tMatrix0_5 = np.dot(tMatrix, np.linalg.inv(tMatrix5_6))
        tMatrix0_4 = np.dot(tMatrix0_5, np.linalg.inv(tMatrix4_5))
        tMatrix1_4 = np.dot(np.linalg.inv(tMatrix0_1), tMatrix0_4)
        p1_4x = tMatrix1_4[0][3]
        p1_4z = tMatrix1_4[2][3]
        p1_4XZ = p1_4x ** 2 + p1_4z ** 2
        a2 = ur5.joint2.a
        a3 = ur5.joint3.a
        theta3 = np.arccos((p1_4XZ - (a2 ** 2) - (a3 ** 2)) / (2 * a2 * a3))
        theta2 = np.arctan2(-p1_4z, -p1_4x) - np.arcsin((-a3*np.sin(theta3))/(p1_4XZ**0.5))
        tMatrix1_2 = self.generateTMatrix(theta2, dh[1]['distance'], dh[1]['a'], dh[1]['alpha'])
        tMatrix2_3 = self.generateTMatrix(theta3, dh[2]['distance'], dh[2]['a'], dh[2]['alpha'])
        tMatrix1_3 = np.dot(tMatrix1_2, tMatrix2_3)
        tMatrix0_3 = np.dot(tMatrix0_1, tMatrix1_3)
        tMatrix3_4 = np.dot(np.linalg.inv(tMatrix0_3), tMatrix0_4)
        x3_4x = tMatrix3_4[0][0]
        x3_4y = tMatrix3_4[1][0]
        theta4 = np.arctan2(x3_4y,x3_4x)
        if not np.isnan(theta2):
            tMatrix = self.forwardKinematic(theta1, theta2, theta3, theta4, theta5, theta6)
            angles = [theta1, theta2, theta3, theta4, theta5, theta6]
            print(angles)
            print(tMatrix)
            return angles
        return []

    def createCPT(self,angles):
        totalTime = 2
        q0 = 0
        resultsTPC = []
        times = np.arange(0, totalTime, 0.01)
        for joint in angles:
            qf = joint
            positions= []
            velocitys = [] 
            accelarations = []
            for t in times:
                jointPosition = 3*(qf/4)*(t**2) - (qf/4)*(t**3)
                jointVelocity = (3/2)*qf*t - (3/4)*qf*(t**2)
                jointAcceleration = (3/2)*qf - (3/4)*qf*t
                positions.append(jointPosition)
                velocitys.append(jointVelocity)
                accelarations.append(jointAcceleration)
            resultsTPC.append(
                {   'qf': qf, 
                    'positions': positions,
                    'velocitys': velocitys,
                    'acceletations': accelarations,
                    'time': np.arange(0, totalTime, 0.01),
                    'q0': q0,
                    }
                )
        return resultsTPC

    def createLSPB(self,angles):
        resultsLSPB = []
        for joint in angles:
            qf = joint
            V = 1.5*(qf - q0)/totalTime
            tb = (q0 - qf + V*totalTime)/V
            alpha = V/tb
            positions= []
            velocitys = [] 
            accelarations = []
            for t in times:
                if t<=tb:
                    jointPosition = q0 + (alpha*(t**2))/2
                    jointVelocity = alpha*t
                    joitAcceleration = alpha
                    positions.append(jointPosition)
                    velocitys.append(jointVelocity)
                    accelarations.append(joitAcceleration)
                elif tb < t and totalTime - tb >= t:
                    jointPosition = (qf+q0 - V*totalTime)/2 + V*t
                    jointVelocity = V
                    joitAcceleration = alpha
                    positions.append(jointPosition)
                    velocitys.append(jointVelocity)
                    accelarations.append(joitAcceleration)

                else:
                    jointPosition = qf - alpha*(totalTime**2)/2 + alpha*totalTime*t - (alpha*(t**2))/2
                    jointVelocity = alpha*totalTime - alpha*t
                    jointAcceleration = -alpha
                    positions.append(jointPosition)
                    velocitys.append(jointVelocity)
                    accelarations.append(jointAcceleration)
                resultsLSPB.append(
                {
                    'positions': positions,
                    'velocitys': velocitys,
                    'acceletations': accelarations,
                    'time': np.arange(0, totalTime, 0.01),
                    'q0': q0,
                    }
                )
        return resultsLSPB

    def run(self):
        i = 0
        while self.step(self.timeStep) != -1:
            i = i + 0.1
            self.shoulder_pan_joint.setPosition(i)
        
    def goToThePoint(self, point):
        x = point[0]
        y = point[1]
        z = point[2]
        finalAngles = self.invKinematic(x, y, z)
        resultsLSPB = self.createLSPB(finalAngles)
        print(resultsLSPB)

        