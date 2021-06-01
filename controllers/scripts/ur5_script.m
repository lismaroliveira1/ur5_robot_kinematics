%{

ur5 equations
UR5 DH paramenters from https: // www.universal - robots.com / articles / ur / application - installation / dh - parameters - for - calculations - of - kinematics - and - dynamics /
UR5e
Kinematics | theta [rad] | a [m] | d [m] | alpha [rad] | Dynamics | Mass [kg] | Center of Mass [m]
Joint 1 | 0 | 0 | 0.1625 | π / 2 | Link 1 | 3.761 | [0, -0.02561, 0.00193]
Joint 2 | 0 |- 0.425 | 0 | 0 | Link 2 | 8.058 | [0.2125, 0, 0.11336]
Joint 3 | 0 | -0.3922 | 0 | 0 | Link 3 | 2.846 | [0.15, 0.0, 0.0265]
Joint 4 | 0 | 0 | 0.1333 | π / 2 | Link 4 | 1.37 | [0, -0.0018, 0.01634]
Joint 5 | 0 | 0 | 0.0997 |- π / 2 | Link 5 | 1.3 | [0, 0.0018, 0.01634]
Joint 6 | 0 | 0 | 0.0996 | 0 | Link 6 | 0.365 | [0, 0, -0.001159]
%}

theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;
theta5 = 0;
theta6 = 0;

a1 = 0;
a2 = 0;
a3 = -0.425;
a4 = -0.3922;
a5 = 0;
a6 = 0;

alpha1 = 0;
alpha2 = pi / 2;
alpha3 = 0;
alpha4 = 0;
alpha5 = (pi / 2);
alpha6 = -(pi / 2);

distance1 = 0.1625;
distance2 = 0;
distance3 = 0;
distance4 = 0.1333;
distance5 = 0.0997;
distance6 = 0.0996;

dhParameters = [
            theta1 a1 distance1 alpha1;
            theta2 a2 distance2 alpha2;
            theta3 a3 distance3 alpha3;
            theta4 a4 distance4 alpha4;
            theta5 a5 distance5 alpha5;
            theta6 a6 distance6 alpha6;
            ];

tMatrixArray = {};

for i = 1:6

    theta = dhParameters(i, 1);
    a = dhParameters(i, 2);
    distance = dhParameters(i, 3);
    alpha = dhParameters(i, 4);

    tMatrix = [
            cos(theta) -sin(theta) 0 a;
            sin(theta) * cos(alpha) cos(theta) * cos(alpha) -sin(alpha) -sin(alpha) * distance;
            sin(theta) * sin(alpha) cos(theta) * sin(alpha) cos(alpha) cos(alpha) * distance;
            0 0 0 1;
            ]
    tMatrixArray{i} = tMatrix;

end

tMatrix0_6 = tMatrixArray{1} * tMatrixArray{2} * tMatrixArray{3} * tMatrixArray{4} * tMatrixArray{5} * tMatrixArray{6}
