import numpy as np
from controller import Robot
import matplotlib.pyplot as plt
import time

pi = np.pi


class Ur5Controller(Robot):
    def __init__(self, ur5):
        super(Ur5Controller, self).__init__()
        self.gripper_gps = None
        self.timeStep = 32
        self.ur5 = ur5

        # Instantiating the motors class for every UR5 joint
        self.elbow_joint = self.getDevice('elbow_joint')
        self.shoulder_lift_joint = self.getDevice('shoulder_lift_joint')
        self.shoulder_pan_joint = self.getDevice('shoulder_pan_joint')
        self.wrist_1_joint = self.getDevice('wrist_1_joint')
        self.wrist_2_joint = self.getDevice('wrist_2_joint')
        self.wrist_3_joint = self.getDevice('wrist_3_joint')

        # Instantiating the PositionSensor class for every UR5 joint
        self.elbow_joint_sensor = self.getDevice('elbow_joint_sensor')
        self.shoulder_lift_joint_sensor = self.getDevice(
            'shoulder_lift_joint_sensor')
        self.shoulder_pan_joint_sensor = self.getDevice(
            'shoulder_pan_joint_sensor')
        self.wrist_1_joint_sensor = self.getDevice('wrist_1_joint_sensor')
        self.wrist_2_joint_sensor = self.getDevice('wrist_2_joint_sensor')
        self.wrist_3_joint_sensor = self.getDevice('wrist_3_joint_sensor')



        # Instantiating the motors class for every gripper joint
        self.palm_finger_1_joint = self.getDevice('palm_finger_1_joint')
        self.finger_1_joint_1 = self.getDevice('finger_1_joint_1')
        self.finger_1_joint_2 = self.getDevice('finger_1_joint_2')
        self.finger_1_joint_3 = self.getDevice('finger_1_joint_3')
        self.palm_finger_2_joint = self.getDevice('palm_finger_2_joint')
        self.finger_2_joint_1 = self.getDevice('finger_2_joint_1')
        self.finger_2_joint_2 = self.getDevice('finger_2_joint_2')
        self.finger_2_joint_3 = self.getDevice('finger_2_joint_3')
        self.finger_middle_joint_1 = self.getDevice('finger_middle_joint_1')
        self.finger_middle_joint_2 = self.getDevice('finger_middle_joint_2')
        self.finger_middle_joint_3 = self.getDevice('finger_middle_joint_3')

        # Instantiating the position class for every gripper joint
        self.palm_finger_1_joint_sensor = self.getDevice('palm_finger_1_joint_sensor')
        self.finger_1_joint_1_sensor = self.getDevice('finger_1_joint_1_sensor')
        self.finger_1_joint_2_sensor = self.getDevice('finger_1_joint_2_sensor')
        self.finger_1_joint_3_sensor = self.getDevice('finger_1_joint_3_sensor')
        self.palm_finger_2_joint_sensor = self.getDevice('palm_finger_2_joint_sensor')
        self.finger_2_joint_1_sensor = self.getDevice('finger_2_joint_1_sensor')
        self.finger_2_joint_2_sensor = self.getDevice('finger_2_joint_2_sensor')
        self.finger_2_joint_3_sensor = self.getDevice('finger_2_joint_3_sensor')
        self.finger_middle_joint_1_sensor = self.getDevice('finger_middle_joint_1_sensor')
        self.finger_middle_joint_2_sensor = self.getDevice('finger_middle_joint_2_sensor')
        self.finger_middle_joint_3_sensor = self.getDevice('finger_middle_joint_3_sensor')

    def initUr5(self, gripper_gps=None):
        # Activating the sensors
        self.elbow_joint_sensor.enable(self.timeStep)
        self.shoulder_lift_joint_sensor.enable(self.timeStep)
        self.shoulder_pan_joint_sensor.enable(self.timeStep)
        self.wrist_1_joint_sensor.enable(self.timeStep)
        self.wrist_2_joint_sensor.enable(self.timeStep)
        self.wrist_3_joint_sensor.enable(self.timeStep)

        # Initial position all theta joints is equal zero
        self.elbow_joint.setPosition(0)
        self.shoulder_lift_joint.setPosition(0)
        self.shoulder_pan_joint.setPosition(0)
        self.wrist_1_joint.setPosition(0)
        self.wrist_2_joint.setPosition(0)
        self.wrist_3_joint.setPosition(0)

        # Enable position sensor
        self.palm_finger_1_joint_sensor.enable(self.timeStep)
        self.finger_1_joint_1_sensor.enable(self.timeStep)
        self.finger_1_joint_2_sensor.enable(self.timeStep)
        self.finger_1_joint_3_sensor.enable(self.timeStep)
        self.palm_finger_2_joint_sensor.enable(self.timeStep)
        self.finger_2_joint_1_sensor.enable(self.timeStep)
        self.finger_2_joint_2_sensor.enable(self.timeStep)
        self.finger_2_joint_3_sensor.enable(self.timeStep)
        self.finger_middle_joint_1_sensor.enable(self.timeStep)
        self.finger_middle_joint_2_sensor.enable(self.timeStep)
        self.finger_middle_joint_3_sensor.enable(self.timeStep)
        print(self.finger_middle_joint_3_sensor.getValue())

        if gripper_gps is not None:
            self.gripper_gps = gripper_gps
            self.gripper_gps.enable(1)
            value = self.gripper_gps.getValues()
            print(value)

    def dhParams(self):
        dh = [
            {
                "alpha": 0,
                "a": 0,
                "distance": self.ur5.joint1.distance,
                "theta": self.ur5.joint1.theta,
            },
            {
                "alpha": self.ur5.joint1.alpha,
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

    @staticmethod
    def generateTMatrix(theta, distance, a, alpha):
        matrix = np.array(
            [[np.cos(theta), -np.sin(theta), 0, a],
             [np.sin(theta) * np.cos(alpha),
              np.cos(theta) * np.cos(alpha),
              - np.sin(alpha), -np.sin(alpha) * distance],
             [np.sin(theta) * np.sin(alpha),
              np.cos(theta) * np.sin(alpha),
              np.cos(alpha), np.cos(alpha) * distance],
             [0, 0, 0, 1]])
        return matrix

    def forwardKinematic(self, angle_joint1, angle_joint2, angle_joint3, angle_joint4, angle_joint5, angle_joint6):
        self.ur5.updateJointsPositions(angle_joint1, angle_joint2, angle_joint3, angle_joint4, angle_joint5,
                                       angle_joint6)
        dh = self.dhParams()
        t_matrix0_6 = np.identity(4)
        for joint in dh:
            t_matrix = self.generateTMatrix(
                joint['theta'], joint['distance'], joint['a'], joint['alpha'])
            t_matrix0_6 = np.dot(t_matrix0_6, t_matrix)
        return t_matrix0_6

    def inv_kinematic(self, x, y, z):
        dh = self.dhParams()
        position = np.array([[0, 0, 1, x], [-1, 0, 0, y], [0, -1, 0, z], [0, 0, 0, 1]])
        p0_5 = np.dot(position, np.array([[0], [0], [-self.ur5.joint6.distance], [1]]))
        p0_5x = p0_5[0][0]
        p0_5y = p0_5[1][0]
        psi = np.arctan2(p0_5y, p0_5x)
        phi = np.arccos(dh[3]['distance'] / ((p0_5x ** 2 + p0_5y ** 2) ** (1 / 2)))
        theta1 = psi - phi + pi / 2
        # self.shoulder_pan_joint.setPosition(theta1)
        p0_6x = position[0][3]
        p0_6y = position[1][3]
        theta5_1 = -np.arccos((p0_6x * np.sin(theta1) - p0_6y * np.cos(theta1) - dh[3]['distance']) / dh[5]['distance'])
        theta5_2 = +np.arccos((p0_6x * np.sin(theta1) - p0_6y * np.cos(theta1) - dh[3]['distance']) / dh[5]['distance'])
        theta5 = max(theta5_1, theta5_2)
        xoy = position[0][1]
        yoy = position[1][1]
        xox = position[0][0]
        yox = position[1][0]
        theta6 = np.arctan2((-xoy * np.sin(theta1) + yoy * np.cos(theta1)) / np.sin(theta5),
                            (xox * np.sin(theta1) - yox * np.cos(theta1)) / np.sin(theta5))
        t_matrix0_1 = self.generateTMatrix(theta1, dh[0]['distance'], dh[0]['a'], dh[0]['alpha'])
        t_matrix4_5 = self.generateTMatrix(theta5, dh[4]['distance'], dh[4]['a'], dh[4]['alpha'])
        t_matrix5_6 = self.generateTMatrix(theta6, dh[5]['distance'], dh[5]['a'], dh[5]['alpha'])
        t_matrix0_5 = np.dot(position, np.linalg.inv(t_matrix5_6))
        t_matrix0_4 = np.dot(4, np.linalg.inv(t_matrix4_5))
        t_matrix1_4 = np.dot(np.linalg.inv(t_matrix0_1), t_matrix0_4)
        p1_4x = t_matrix1_4[0][3]
        p1_4z = t_matrix1_4[2][3]
        p1_4_xz = np.sqrt(p1_4x ** 2 + p1_4z ** 2)
        a2 = self.ur5.joint2.a
        a3 = self.ur5.joint3.a
        theta3_1 = None
        theta3_2 = None
        try:
            theta3_1 = np.arccos((p1_4_xz ** 2 - (a2 ** 2) - (a3 ** 2)) / (2 * a2 * a3))
            theta3_2 = np.arccos((p1_4_xz ** 2 - (a2 ** 2) - (a3 ** 2)) / (2 * a2 * a3))
        except:
            pass
        if theta3_2 is None and theta3_1 is None:
            return
        theta3 = max(theta3_1, theta3_2)
        theta2 = np.arctan2(-p1_4z, -p1_4x) - np.arcsin((-a3 * np.sin(theta3)) / p1_4_xz)
        t_matrix1_2 = self.generateTMatrix(theta2, dh[1]['distance'], dh[1]['a'], dh[1]['alpha'])
        t_matrix2_3 = self.generateTMatrix(theta3, dh[2]['distance'], dh[2]['a'], dh[2]['alpha'])
        t_matrix1_3 = np.dot(t_matrix1_2, t_matrix2_3)
        t_matrix0_3 = np.dot(t_matrix0_1, t_matrix1_3)
        t_matrix3_4 = np.dot(np.linalg.inv(t_matrix0_3), t_matrix0_4)
        x3_4x = t_matrix3_4[0][0]
        x3_4y = t_matrix3_4[1][0]
        theta4 = np.arctan2(x3_4y, x3_4x)
        if not np.isnan(theta2):
            t_matrix = self.forwardKinematic(theta1, theta2, theta3, theta4, theta5, theta6)
            print(t_matrix)
            angles = [theta1, theta2, theta3, theta4, theta5, theta6]
            return angles
        return []

    @staticmethod
    def create_cpt(angles):
        total_time = 2
        q0 = 0
        results_tpc = []
        times = np.arange(0, total_time, 0.01)
        for joint in angles:
            qf = joint
            positions = []
            speeds = []
            accelerations = []
            for t in times:
                joint_position = 3 * (qf / 4) * (t ** 2) - (qf / 4) * (t ** 3)
                joint_velocity = (3 / 2) * qf * t - (3 / 4) * qf * (t ** 2)
                joint_acceleration = (3 / 2) * qf - (3 / 4) * qf * t
                positions.append(joint_position)
                speeds.append(joint_velocity)
                accelerations.append(joint_acceleration)
            results_tpc.append(
                {'qf': qf,
                 'positions': positions,
                 'speeds': speeds,
                 'accelerations': accelerations,
                 'time': np.arange(0, total_time, 0.01),
                 'q0': q0,
                 }
            )
        return results_tpc

    @staticmethod
    def create_lspb(angles, total_time):
        results_lspb = []
        q0 = 0
        times = np.arange(0, total_time, 0.01)
        i = 0
        names = ['Shoulder Pan Joint', 'Shoulder Lift Joint', 'Elbow Joint', 'Wrist 1 Joint',
                 'Wrist 1 Joint', 'Wrist 1 Joint']
        for joint in angles:
            qf = joint
            v = 1.5 * (qf - q0) / total_time
            tb = (q0 - qf + v * total_time) / v
            alpha = v / tb
            positions = []
            speeds = []
            accelerations = []
            for t in times:
                if t <= tb:
                    joint_position = q0 + (alpha * (t ** 2)) / 2
                    joint_velocity = alpha * t
                    joint_acceleration = alpha
                    positions.append(joint_position)
                    speeds.append(joint_velocity)
                    accelerations.append(joint_acceleration)
                elif tb < t <= total_time - tb:
                    joint_position = (qf + q0 - v * total_time) / 2 + v * t
                    joint_velocity = v
                    joint_acceleration = alpha
                    positions.append(joint_position)
                    speeds.append(joint_velocity)
                    accelerations.append(joint_acceleration)
                else:
                    joint_position = qf - alpha * (total_time ** 2) / \
                                     2 + alpha * total_time * t - (alpha * (t ** 2)) / 2
                    joint_velocity = alpha * total_time - alpha * t
                    joint_acceleration = -alpha
                    positions.append(joint_position)
                    speeds.append(joint_velocity)
                    accelerations.append(joint_acceleration)
            results_lspb.append(
                {
                    'positions': positions,
                    'speeds': speeds,
                    'accelerations': accelerations,
                    'time': np.arange(0, total_time, 0.01),
                    'q0': q0,
                    'name': names[i]
                }
            )
            i += 1
        return results_lspb

    def run(self):
        i = 0
        while self.step(self.timeStep) != -1:
            i = i + 0.1
            self.shoulder_pan_joint.setPosition(i)

    def go_to_goal(self, point):
        x = point[0]
        y = point[1]
        z = point[2]
        final_angles = self.inv_kinematic(x, y, z)
        self.goToPosition(final_angles, 1.5)
        results_lspb = self.create_lspb(final_angles, 1)
        i = 0
        if results_lspb:
            plt.figure(figsize=(9, 6))
            plt.title('Positions')
            for i in range(len(results_lspb)):
                lines = plt.plot(results_lspb[i]['time'], results_lspb[i]['positions'])
                plt.setp(lines, linewidth=1.0, label=results_lspb[i]['name'])
                plt.legend()
            plt.grid(linestyle='-', linewidth=0.5)
            plt.show()

            plt.figure(figsize=(9, 6))
            plt.title('Velocity')
            for a in range(len(results_lspb)):
                lines = plt.plot(results_lspb[a]['time'], results_lspb[a]['speeds'])
                plt.setp(lines, linewidth=1.0, label=results_lspb[i]['name'])
                plt.legend()
            plt.grid(linestyle='-', linewidth=0.5)
            plt.show()

            plt.figure(figsize=(9, 6))
            plt.title('Acceleration')
            for a in range(len(results_lspb)):
                lines = plt.plot(results_lspb[a]['time'], results_lspb[a]['accelerations'])
                plt.setp(lines, linewidth=1.0, label=results_lspb[i]['name'])
                plt.legend()
            plt.grid(linestyle='-', linewidth=0.5)
            plt.show()

            for result in results_lspb:
                for angle in result['positions']:
                    if i == 0:
                        self.step(1)
                        self.shoulder_pan_joint.setPosition(angle)
                    if i == 1:
                        self.step(1)
                        self.shoulder_lift_joint.setPosition(angle)
                    if i == 2:
                        self.step(1)
                        self.elbow_joint.setPosition(angle)
                    if i == 3:
                        self.step(1)
                        self.wrist_1_joint.setPosition(angle)
                    if i == 4:
                        self.step(1)
                        self.wrist_2_joint.setPosition(angle)
                    if i == 5:
                        self.step(1)
                        self.wrist_3_joint.setPosition(angle)
            plt.figure(figsize=(9, 3))
            plt.suptitle('Position')

    def init_gripper(self):
        # Instantiating the motors class for every UR5 joint
        self.palm_finger_1_joint = self.getDevice('palm_finger_1_joint')
        self.finger_1_joint_1 = self.getDevice('finger_1_joint_1')
        self.finger_1_joint_2 = self.getDevice('finger_1_joint_2')
        self.finger_1_joint_3 = self.getDevice('finger_1_joint_3')
        self.palm_finger_2_joint = self.getDevice('palm_finger_2_joint')
        self.finger_2_joint_1 = self.getDevice('finger_2_joint_1')
        self.finger_2_joint_2 = self.getDevice('finger_2_joint_2')
        self.finger_2_joint_3 = self.getDevice('finger_2_joint_3')
        self.finger_middle_joint_1 = self.getDevice('finger_middle_joint_1')
        self.finger_middle_joint_2 = self.getDevice('finger_middle_joint_2')
        self.finger_middle_joint_3 = self.getDevice('finger_middle_joint_3')

    def open_gripper(self):
        open_point = 0.05
        tolerance = 0.02
        while self.finger_1_joint_1_sensor.getValue() > open_point + tolerance:
            self.finger_1_joint_1.setPosition(open_point)
            self.finger_2_joint_1.setPosition(open_point)
            self.finger_middle_joint_1.setPosition(open_point)
            self.step(32)

    def close_gripper(self):
        close_point = 1.22
        tolerance = 0.02
        while self.finger_1_joint_1_sensor.getValue() < close_point - tolerance:
            self.finger_1_joint_1.setPosition(close_point)
            self.finger_2_joint_1.setPosition(close_point)
            self.finger_middle_joint_1.setPosition(close_point)
            print(self.finger_1_joint_1_sensor.getValue())
            self.step(32)
        self.finger_1_joint_3_sensor = self.getDevice('finger_1_joint_3_sensor')
        self.palm_finger_2_joint_sensor = self.getDevice('palm_finger_2_joint_sensor')
        self.finger_2_joint_1_sensor = self.getDevice('finger_2_joint_1_sensor')
        self.finger_2_joint_2_sensor = self.getDevice('finger_2_joint_2_sensor')
        self.finger_2_joint_3_sensor = self.getDevice('finger_2_joint_3_sensor')
        self.finger_middle_joint_1_sensor = self.getDevice('finger_middle_joint_1_sensor')
        self.finger_middle_joint_2_sensor = self.getDevice('finger_middle_joint_2_sensor')
        self.finger_middle_joint_3_sensor = self.getDevice('finger_middle_joint_3_sensor')

    def goToPosition(self, positions, velocity):
        joint_sensors = [
            self.shoulder_pan_joint_sensor,
            self.shoulder_lift_joint_sensor,
            self.elbow_joint_sensor,
            self.wrist_1_joint_sensor,
            self.wrist_2_joint_sensor,
            self.wrist_3_joint_sensor
        ]
        joint_motors = [
            self.shoulder_pan_joint,
            self.shoulder_lift_joint,
            self.elbow_joint,
            self.wrist_1_joint,
            self.wrist_2_joint,
            self.wrist_3_joint
        ]
        for i in range(len(positions)):
            joint_position = positions[i]
            joint_motors[i].setVelocity(velocity)
            position = joint_sensors[i].getValue()
            error = abs(joint_position - position)
            while error > 0.02:
                joint_motors[i].setPosition(joint_position)
                position = joint_sensors[i].getValue()
                self.step(10)
            time.sleep(0.5)
