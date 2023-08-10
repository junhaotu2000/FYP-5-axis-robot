clear all
clc
%---------------------------------------------------
% Forward kinematic equations calculated by matrix
%---------------------------------------------------
% Copyright Junhao TU
% Date 2023.05

% Description: This is the calculation code for forward kinematic
%              equations of the designed 5-axis robot using matrix

% Input: Rotational angle of 5 joints
% Output: Coordinates of end point
%-----------------------------------------------------------------

% Define the values of 5 rotational angles
rotation = [0, 0, 0, 0, 0];

% Define the position of end effector
P = [0;0;0;1];

%-------- Define the DH parameters for 5 joints-------------
% Define values of theta
theta1 = ((rotation(1)*pi)/180);
theta2 = -pi/2 + ((rotation(2)*pi)/180);
theta3 = ((rotation(3)*pi)/180);
theta4 = ((rotation(4)*pi)/180);
theta5 = pi/2 + ((rotation(5)*pi)/180);

% Define values of d
d1 = 261;
d2 = 0;
d3 = 0;
d4 = 223;
d5 = 0;

% Define values of a
a1 = 0;
a2 = 220;
a3 = 0;
a4 = 0;
a5 = 158;

% Define values of alpha
alpha1 = -pi/2;
alpha2 = 0;
alpha3 = -pi/2;
alpha4 = pi/2;
alpha5 = 0;

%----------------Calculate the transformation matrix---------------
T1 = [cos(theta1),-cos(alpha1)*sin(theta1),sin(alpha1)*sin(theta1),a1*cos(theta1);...
      sin(theta1),cos(alpha1)*cos(theta1),-sin(alpha1)*cos(theta1),a1*sin(theta1);...
      0,sin(alpha1),cos(alpha1),d1;0,0,0,1];
T2 = [cos(theta2),-cos(alpha2)*sin(theta2),sin(alpha2)*sin(theta2),a2*cos(theta2);...
      sin(theta2),cos(alpha2)*cos(theta2),-sin(alpha2)*cos(theta2),a2*sin(theta2);...
      0,sin(alpha2),cos(alpha2),d2;0,0,0,1];
T3 = [cos(theta3),-cos(alpha3)*sin(theta3),sin(alpha3)*sin(theta3),a3*cos(theta3);...
      sin(theta3),cos(alpha3)*cos(theta3),-sin(alpha3)*cos(theta3),a3*sin(theta3);...
      0,sin(alpha3),cos(alpha3),d3;0,0,0,1];
T4 = [cos(theta4),-cos(alpha4)*sin(theta4),sin(alpha4)*sin(theta4),a4*cos(theta4);...
      sin(theta4),cos(alpha4)*cos(theta4),-sin(alpha4)*cos(theta4),a4*sin(theta4);...
      0,sin(alpha4),cos(alpha4),d4;0,0,0,1];
T5 = [cos(theta5),-cos(alpha5)*sin(theta5),sin(alpha5)*sin(theta5),a5*cos(theta5);...
      sin(theta5),cos(alpha5)*cos(theta5),-sin(alpha5)*cos(theta5),a5*sin(theta5);...
      0,sin(alpha5),cos(alpha5),d5;0,0,0,1];

%---------Calculate the coordinates of end effector----------------
T = T1*T2*T3*T4*T5*P

location = [T(1) T(2) T(3)]