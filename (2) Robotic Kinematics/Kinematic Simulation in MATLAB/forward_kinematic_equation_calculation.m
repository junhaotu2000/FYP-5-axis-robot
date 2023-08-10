clear all
clc
%---------------------------------------------------
% Forward kinematic equations calculated by equations
%---------------------------------------------------
% Copyright Junhao TU
% Date 2023.05

% Description: This is the calculation code for coordinates of end
%              point using equations

% Input: Rotational angle of 5 joints
% Output: Coordinates of end point
%-----------------------------------------------------------------

% Define the values of 5 rotational angles
angle = [0, 0, 0, 0, 0]

st1 = angle(1)*pi/180;
st2 = angle(2)*pi/180;
st3 = angle(3)*pi/180;
st4 = angle(4)*pi/180;
st5 = angle(5)*pi/180;

% Define the configuration parameters
L1 = 261
L2 = 220
L3 = 223
L4 = 158

%---------------Calculate the coordintes of end point------------------
x = L2*cos(st1)*sin(st2) - L4*sin(st5)*(sin(st1)*sin(st4)+cos(st4)* ...
    (cos(st1)*sin(st2+st3))) + (L4*cos(st5)+L3)*(cos(st1)*cos(st2+st3))
y = L2*sin(st1)*sin(st2) - L4*sin(st5)*(cos(st1)*sin(st4)+cos(st4)* ...
    (sin(st1)*sin(st2+st3))) + (L4*cos(st5)+L3)*(sin(st1)*cos(st2+st3))
z = L2*cos(st2) - L3*sin(st2+st3) - L4*cos(st5)*sin(st2+st3) - ...
    L4*cos(st4)*sin(st5)*cos(st2+st3) + L1

location = [x y z]