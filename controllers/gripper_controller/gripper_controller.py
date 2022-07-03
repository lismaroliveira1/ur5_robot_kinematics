from controller import Robot
from enum import Enum


class GripperState(Enum):
    WAITING = 0
    GRASPING = 1
    ROTATING = 2
    RELEASING = 3
    ROTATING_BACK = 4


class GripperController(Robot):
    def __init__(self):
        self.timeStep = 10
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

    def initGripper(self):
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

    def open_gripper(self):
        pass

    def close_gripper(self):
        while self.getDevice('palm_finger_1_joint_sensor') < 0.05:
            self.palm_finger_1_joint.setPosition(self.getDevice('palm_finger_1_joint_sensor') - 0.05)
            self.step(1)

        finger_1_joint_1_sensor = self.getDevice('finger_1_joint_1_sensor')
        finger_1_joint_2_sensor = self.getDevice('finger_1_joint_2_sensor')
        finger_1_joint_3_sensor = self.getDevice('finger_1_joint_3_sensor')
        palm_finger_2_joint_sensor = self.getDevice('palm_finger_2_joint_sensor')
        finger_2_joint_1_sensor = self.getDevice('finger_2_joint_1_sensor')
        finger_2_joint_2_sensor = self.getDevice('finger_2_joint_2_sensor')
        finger_2_joint_3_sensor = self.getDevice('finger_2_joint_3_sensor')
        finger_middle_joint_1_sensor = self.getDevice('finger_middle_joint_1_sensor')
        finger_middle_joint_2_sensor = self.getDevice('finger_middle_joint_2_sensor')
        finger_middle_joint_3_sensor = self.getDevice('finger_middle_joint_3_sensor')

    def goToHomePosition(self):
        palm_finger_1_joint_sensor = self.palm_finger_1_joint_sensor.getValue()
        self.palm_finger_1_joint.setPosition(0.1)
        self.step(1)
        pass
