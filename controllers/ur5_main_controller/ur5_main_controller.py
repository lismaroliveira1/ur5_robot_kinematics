# -*- coding: utf-8 -*-
from joint import Joint
from ur5e import UR5e
from ur5_controller import Ur5Controller

import math


#######   main   ######


""" 

UR5 DH Paramaters
Kinematic | theta |   a[m]   |   d[m] | alpha |         Device       | Mass [kg] |  Center of Mass [kh] 
Joint 1   |   0   |     0    | 0.1625 |  pi/2 |  Shoulder pan joint  |   3.761   | [0, -0.02561, 0.00193]
Joint 2   |   0   |  -0.425  |   0    |    0  |  Shoulder lift joint |   8.058   |  [0.2125, 0, 0.11336]
Joint 3   |   0   |  -0.3922 |   0    |   0   |     Elbow joint      |   2.846   |  [0.15, 0.0, 0.0265]
Joint 4   |   0   |     0    | 0.1333 |  pi/2 |    Wrist 1 joint     |   1.37    | [0, -0.0018, 0.01634]
Joint 5   |   0   |     0    | 0.0997 | -pi/2 |    Wrist 2 joint     |   1.3     |  [0, 0.0018,0.01634]
Joint 6   |   0   |     0    | 0.0996 |   0   |    Wrist 3 joint     |   0.365   |   [0, 0, -0.001159]

UR5 DH Table

  i |   alpha(i-1)    |   a(i-1) |   d(i) | theta(i)    
  1 |        0        |     0    | 0.1625 | theta(1)
  2 |       pi/2      |     0    |   0    | theta(2)
  3 |        0        |  -0.425  |   0    | theta(3)
  4 |        0        |  -0.3922 | 0.1333 | theta(4)
  5 |      pi/2       |     0    | 0.0997 | theta(5)
  6 |     -pi/2       |     0    | 0.0996 | theta(6)

 """

joint1 = Joint(0, 0, 0.1625, math.pi/2, 3.761, [[0, -0.02561, 0.00193]])
joint2 = Joint(0, -0.425, 0, 0, 8.058, [0.2125, 0, 0.11336])
joint3 = Joint(0, -0.3922, 0, 0, 2.846, [0.15, 0.0, 0.0265])
joint4 = Joint(0, 0, 0.1333, math.pi/2, 1.37, [0, 0, 0.01])
joint5 = Joint(0, 0, 0.0997, -math.pi/2, 1.3, [0, 0, 0.01])
joint6 = Joint(0, 0, 0.0996, 0, 0.365, [0, 0, -0.001159])


ur5 = UR5e(joint1, joint2, joint3, joint4, joint5, joint6)

controller = Ur5Controller(ur5)

if __name__ == '__main__':
    controller.initUr5()
    controller.forwardKinematic(
        math.pi / 2, -math.pi / 2, -math.pi / 4, math.pi / 2, -math.pi / 4, -math.pi / 2)
    controller.inverseKinematic(
        [0.64, 0, 0], [0, math.pi, 0])
    # controller.inverseKinematic()
