
import numpy as np
from joint import Joint
from ur5e import UR5e
from scipy.spatial.transform import Rotation as R
pi = np.pi

def invKinematic(point):
    for roll in np.arange(0, 360, 1):
        for pitch in np.arange(0, 360, 1):
            for yall in np.arange(0, 360, 1):
                angles = []
                for conv in ['xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx']:
                    try:
                        r = R.from_euler(conv, [0, 0, 0], degrees=True)
                        tMatrix0_6 = np.append(np.vstack([r.as_matrix(), [0,0,0]]), np.array([[x], [y], [z], [1]]), axis=1)
                        p0_6 = np.dot(tMatrix0_6, np.array([[0], [0], [-ur5.joint6.distance], [1]]))
                        p0_5x = p0_6[0][0]
                        p0_5y = p0_6[1][0]
                        theta1 = np.arctan2(p0_5y, p0_5x) + np.arccos(dh[3]['distance'] / ((p0_5x ** 2 + p0_5y ** 2) ** (1 / 2))) + pi / 2
                        p0_6x = tMatrix0_6[0][3]
                        p0_6y = tMatrix0_6[1][3]
                        theta5  = np.arccos((p0_6x * np.sin(theta1) - p0_6y * np.cos(theta1) -  dh[3]['distance']) / dh[5]['distance']) 
                        xoY = tMatrix0_6[0][1]
                        yoY = tMatrix0_6[1][1]
                        xoX = tMatrix0_6[0][0]
                        yoX = tMatrix0_6[1][0]
                        theta6 = np.arctan2((-xoY*np.sin(theta1) + yoY*np.cos(theta1)) / np.sin(theta5),  (xoX*np.sin(theta1) - yoX*np.cos(theta1)) / np.sin(theta5))
                        tMatrix0_1 = generateTMatrix(theta1, dh[0]['distance'], dh[0]['a'], dh[0]['alpha'])
                        tMatrix4_5 = generateTMatrix(theta5, dh[4]['distance'], dh[4]['a'], dh[4]['alpha'])
                        tMatrix5_6 = generateTMatrix(theta6, dh[5]['distance'], dh[5]['a'], dh[5]['alpha'])
                        tMatrix0_5 = np.dot(tMatrix0_6, np.linalg.inv(tMatrix5_6))
                        tMatrix0_4 = np.dot(tMatrix0_5, np.linalg.inv(tMatrix4_5))
                        tMatrix1_4 = np.dot(np.linalg.inv(tMatrix0_1), tMatrix0_4)
                        p1_4x = tMatrix1_4[0][3]
                        p1_4z = tMatrix1_4[2][3]
                        p1_4XZ = (p1_4x ** 2 + p1_4z ** 2)
                        a2 = ur5.joint2.a
                        a3 = ur5.joint3.a
                        theta3 = np.arccos((p1_4XZ**2 - (a2 ** 2) - (a3 ** 2)) / (2 * a2 * a3))
                        theta2 = np.arctan2(-p1_4z, -p1_4x) - np.arcsin((-a3*np.sin(theta3))/p1_4XZ)
                        tMatrix1_2 = generateTMatrix(theta2, dh[1]['distance'], dh[1]['a'], dh[1]['alpha'])
                        tMatrix2_3 = generateTMatrix(theta3, dh[2]['distance'], dh[2]['a'], dh[2]['alpha'])
                        tMatrix1_3 = np.dot(tMatrix1_2, tMatrix2_3)
                        tMatrix0_3 = np.dot(tMatrix0_1, tMatrix1_3)
                        tMatrix3_4 = np.dot(np.linalg.inv(tMatrix0_3), tMatrix0_4)
                        x3_4x = tMatrix3_3[0][0]
                        x3_4y = tMatrix3_3[1][0]
                        theta4 = np.arctan2(x3_4y,x3_4x)
                        angles.append({
                            "theta1": theta1,
                            "theta2": theta2,
                            "theta3": theta3,
                            "theta4": theta4,
                            "theta5": theta5,
                            "theta6": theta6,
                        })
                    except:
                        pass
    return angles

x = 1.5
y = 0.8
z = 0

angles = invKinematic([x, y, z])
print(angles)