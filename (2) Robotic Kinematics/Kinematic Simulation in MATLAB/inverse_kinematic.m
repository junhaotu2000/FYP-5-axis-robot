%---------------------------------------------------------------
% Inverse kinematic clculation of designed robot
%---------------------------------------------------------------
% Copyright Junhao TU
% Data 2023.05

% Description: This is the code to calculate the rotational angles of 
%              3 major joints to reach the desired point in space.
%              Joint 5 rotates to keep the orientation of end effector.

% Input: Desired coordinates of end poing
% Output: Roatational angles of joints
%--------------------------------------------------------------
% Define coordinates of end point
x_end = 381;
y_end = 0;
z_end = 481;

% Define configuration parameters of robot
L1 = 261;
L2 = 220;
L3 = 223;
L4 = 158;

% Define fixed angle of end effector to horizontal axis
fixed_angle = (0)*pi/180

%----------Calculate coordinates of joint 5-----------------
xj5 = x_end - L4*cos(fixed_angle)*(x_end / sqrt(x_end^2 + y_end^2))
yj5 = y_end - L4*cos(fixed_angle)*(y_end / sqrt(x_end^2 + y_end^2))
zj5 = z_end + L4*sin(fixed_angle)

%---------Calculate rotational angles of joints-----------
d1 = sqrt((xj5)^2 + (yj5)^2);
d2 = sqrt((d1)^2 + (zj5)^2)
d3 = sqrt((d1)^2 + (L1 - zj5)^2);

a = atan2(d1, (L1 - zj5));
B = acos(((L2)^2 + (d3)^2 - (L3)^2) / (2*L2*d3));

theta1 = (180*atan2(yj5, xj5))/pi;
theta2 = 180*(pi - a - B)/pi;
theta3 = 180*(pi/2 - acos(((L2)^2 + (L3)^2 - (d3)^2) / (2*L2*L3)))/pi;
theta5 = fixed_angle-(theta2+theta3)


st = [theta1 theta2 theta3 0 theta5]
