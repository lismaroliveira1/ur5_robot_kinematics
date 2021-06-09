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
ur5DH = [
    theta1 distance1 a1 alpha1;
    theta2 distance2 a2 alpha2;
    theta3 distance3 a3 alpha3;
    theta4 distance4 a4 alpha4;
    theta5 distance5 a5 alpha5;
    theta6 distance6 a6 alpha6;
    ];

tMatrixArray = {};

for i = 1:6
    theta = dhParameters(i, 1);
    a = dhParameters(i, 2);
    distance = dhParameters(i, 3);
    alpha = dhParameters(i, 4);

    tMatrix = generateTMatrix(theta, distance, alpha, a);
    tMatrixArray{i} = tMatrix;
end

% Cacluculo da cinem�tica inversa.

x = 1.5;
y = 0.8;
z = 0;

tMatrix0_6 = transl(x, y, z) * rpy2tr(0, 0, 0, 'deg');
p0_5 = tMatrix0_6 * [0; 0; -distance6; 1];
p0_5x = p0_5(1);
p0_5y = p0_5(2);

test1 = atan2(p0_5y, p0_5x) - acos(distance4 / (sqrt(p0_5x * p0_5x + p0_5y * p0_5y))) + pi / 2;
test2 = atan2(p0_5y, p0_5x) + acos(distance4 / (sqrt(p0_5x * p0_5x + p0_5y * p0_5y))) + pi / 2;
theta1 = max(test1, test2)

p0_6x = tMatrix0_6(1, 4);
p0_6y = tMatrix0_6(2, 4);

theta5 = acos((p0_6x * sin(theta1) - p0_6y * cos(theta1) - distance4) / distance6);
test1 = acos((p0_6x * sin(theta1) - p0_6y * cos(theta1) - distance4) / distance6);
test2 = acos((p0_6x * sin(theta1) - p0_6y * cos(theta1) - distance4) / distance6);

x0_6x = tMatrix0_6(1, 1);
x0_6y = tMatrix0_6(1, 2);
y0_6x = tMatrix0_6(2, 1);
y0_6y = tMatrix0_6(2, 2);

theta6 = atan2((-x0_6y * sin(theta1) + y0_6y * cos(theta1)) / sin(theta5), (x0_6x * sin(theta1) - y0_6x * cos(theta1)) / sin(theta5));

tMatrix0_1 = generateTMatrix(theta1, distance1, alpha1, a1);
tMatrix4_5 = generateTMatrix(theta5, distance5, alpha5, a5);
tMatrix5_6 = generateTMatrix(theta6, distance6, alpha6, a6);

tMatrix0_5 = tMatrix0_6 * inv(tMatrix5_6);
tMatrix0_4 = tMatrix0_5 * inv(tMatrix4_5);

tMatrix1_4 = inv(tMatrix0_1) * tMatrix0_4

p1_4x = tMatrix1_4(1, 4);
p1_4z = tMatrix1_4(3, 4);
p1_4xz = sqrt(p1_4x^2 + p1_4z^2)
a2 = -0.425;
a3 = -0.3922;
abs(a2 - a3);
abs(a2 + a3);

theta3 = acos((abs(p1_4xz)^2 - a2^2 - a3^2) / (2 * a2 * a3));

text2 = tMatrix0_1 * tMatrix1_4;

text2 * inv(tMatrix0_4);

posicao = [0 -1 0 1.2; 0.   0.  -1.   1.5; -1.   1.   0.   2.1; 0.   0.   0.   1. ]

ur5Location(posicao)

function saida = ur5Location (posicao)
th = zeros(6,8);
a3 = -0.425;
a4 = -0.39225;

d1 = 0.08916;
d4 = 0.10915;
d5 = 0.09456;
d6 = 0.0823;

P_05 = posicao * [0;0;-d6; 1];

%Calculo de tetha1
  psi = atan2(P_05(2,1),P_05(1,1));
  phi = acos(d4 /sqrt(P_05(2,1)*P_05(2,1) + P_05(1,1)*P_05(1,1)));

  th1 = pi/2 + psi + phi;
  th2 = pi/2 + psi - phi;
  th(1,1) = th1;
  th(2,1) = th2;
  tetha1 = max(th1,th2)
  
%Tetha5

num = posicao(1,4)*sin(tetha1)-posicao(2,4)*cos(tetha1)-d4;
den = d6;

th5_1 = acos(num/den);
th5_2 = -acos(num/den);

tetha5 = max(th5_1,th5_2);

%Tetha6
term1 = -(posicao(1,2))*sin(tetha1) + (posicao(2,2))*cos(tetha1);
tem2 = posicao(1,1)*sin(tetha1) - posicao(2,1)*cos(tetha1);
tetha6 = atan2(term1/sin(tetha5),tem2/sin(tetha5));

%Tetha3

%link5
d(5) = d5; tetha(5) = tetha5; a(5) = 0; alfa(5) = pi/2;
%link6
d(6) = d6; tetha(6) = tetha6; a(6) = 0; alfa(6) = -pi/2;
%link1
d(1) = d1; tetha(1) = tetha1; a(1) = 0; alfa(1) = 0;

%link1
T01 = dh_ur5(a(1),alfa(1),d(1),tetha(1));
%link5
T45 = dh_ur5(a(5),alfa(5),d(5),tetha(5));
%link6
T56 = dh_ur5(a(6),alfa(6),d(6),tetha(6));

T05 = posicao*inv(T56);
T04 = T05*inv(T45);
T14 = inv(T01)*T04;

p14x = T14(1,4);
p14z = T14(3,4);
p14xz = p14x^2+p14z^2;
th3_1 = acos(((p14xz-a3^2-a4^2)/(2*a3*a4)));
th3_2 = -acos(((p14xz-a3^2-a4^2)/(2*a3*a4)));

tetha3 = max(th3_1,th3_2);

%Tetha2

tetha2 = atan2(-T14(3,4),-T14(1,4))-asin((-a4*sin(tetha3))/(sqrt(p14xz)));

%tetha4

%link2
d(2) = 0; tetha(2) = tetha2; a(2) = 0; alfa(2) = pi/2;
%link3
d(3) = 0; tetha(3) = tetha3; a(3) = a3; alfa(3) = 0;

T12 = dh_ur5(a(2),alfa(2),d(2),tetha(2));
T23 = dh_ur5(a(3),alfa(3),d(3),tetha(3));
T24 = T12\T14;
T34 = T23\T24;

tetha4 = atan2(T34(2,1),T34(1,1));

saida = rad2deg([tetha1 tetha2 tetha3 tetha4 tetha5 tetha6]);
end



function tMatrix = generateTMatrix(theta, distance, alpha, a)
    tMatrix = [
            cos(theta) -sin(theta) 0 a;
            sin(theta) * cos(alpha) cos(theta) * cos(alpha) -sin(alpha) -sin(alpha) * distance;
            sin(theta) * sin(alpha) cos(theta) * sin(alpha) cos(alpha) cos(alpha) * distance;
            0 0 0 1;
            ];
end
