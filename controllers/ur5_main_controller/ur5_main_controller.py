from joint import Joint
from ur5e import UR5e
from ur5_controller import Ur5Controller
import math


#######   main   ######
object_point = [0, 0.823, -0.68]

joint1 = Joint(0, 0, 0.1519, math.pi/2, 2, [0, -0.02, 0])
joint2 = Joint(0, -0.24365, 0, 0, 3.42, [0.13, 0, 0.0238])
joint3 = Joint(0, -0.21325, 0, 0, 1.26, [0.05, 0, 0.0238])
joint4 = Joint(0, 0, 0.11235, math.pi/2, 0.8, [0, 0, 0.01])
joint5 = Joint(0, 0, 0.08535, -math.pi/2, 0.8, [0, 0, 0.01])
joint6 = Joint(0, 0, 0.0819, 0, 0.35, [0, 0, 0.01])

ur5 = UR5e(joint1, joint2, joint3, joint4, joint5, joint6)

controller = Ur5Controller(ur5)

if __name__ == '__main__':
    controller.initUr5()
    controller.forwardKinematic()
    controller.inverseKinematic()
    controller.run()
