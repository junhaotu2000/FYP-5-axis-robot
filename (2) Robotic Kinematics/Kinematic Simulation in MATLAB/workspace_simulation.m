clear all
clc
%---------------------------------------------------------------
% Use Monte Carlo method to simulate workspace of designed robot
%---------------------------------------------------------------
% Copyright Junhao TU
% Data 2023.05

% Description: This is the code to simulate the worspce of designed
%              robot based on forward kinematics using Monto Carlo method

% Output: Simulated workspce in 3D plane
%--------------------------------------------------------------

% Define transformation and plotting parameters
radian = pi/180;
fontsize = 9;
titlesize = 9.1;

% ----------Define DH Parameters of Designed Robot-------------
% Define values of d
d1 = 261;
d2 = 0;
d3 = 0;
d4 = 223;
d5 = 0;

% Desine values of a
a1 = 0;
a2 = 220;
a3 = 0;
a4 = 0;
a5 = 158;
 
% Define values of alpha
alpha1 = -90 * radian;
alpha2 =   0 * radian;
alpha3 = -90 * radian;
alpha4 = 90 * radian;
alpha5 =  0 * radian;

% -----------------------Define angle limits--------------------------
lim1_min = -160 * radian; lim1_max = 160 * radian; % Joint 1(-160，160)
lim2_min = -140 * radian; lim2_max = 140 * radian; % Joint 2(-140，140)
lim3_min = -230 * radian; lim3_max = 50 * radian; % Joint 3(-230，50)
lim4_min = -140 * radian; lim4_max = 140 * radian; % Joint 4(-140，140)
lim5_min = -140 * radian; lim5_max = 140 * radian; % JOint 5(-140，140)
 
% Calculate rotation angle ranges
lim1 = lim1_max - lim1_min;
lim2 = lim2_max - lim2_min;
lim3 = lim3_max - lim3_min;
lim4 = lim4_max - lim4_min;
lim5 = lim5_max - lim5_min;
 
% --------------------Define different links------------------------
%           θ   d    a    α                     joint angle limits                       
L(1)=Link([ 0   d1   a1   alpha1], 'standard'); L(1).qlim=[lim1_min,lim1_max];
L(2)=Link([ 0   d2   a2   alpha2], 'standard'); L(2).qlim=[lim2_min,lim2_max]; 
L(3)=Link([ 0   d3   a3   alpha3], 'standard'); L(3).qlim=[lim3_min,lim3_max]; 
L(4)=Link([ 0   d4   a4   alpha4], 'standard'); L(4).qlim=[lim4_min,lim4_max];
L(5)=Link([ 0   d5   a5   alpha5], 'standard'); L(5).qlim=[lim5_min,lim5_max]; 
 
% Define joint offsets
L(2).offset=-pi/2;
L(5).offset=pi/2

% ---------------Establish the robotic arm------------------
fivelink=SerialLink(L,'name','Mr. Liu');

% Display parameters in command window
fivelink.display();
 
% Use Monte Carlo method to generate random number of angles within limits
N = 20000;
theta1 = ( lim1_min + (lim1 * rand(N,1)) );
theta2 = ( lim2_min + (lim2 * rand(N,1)) );
theta3 = ( lim3_min + (lim3 * rand(N,1)) );
theta4 = ( lim4_min + (lim4 * rand(N,1)) );
theta5 = ( lim5_min + (lim5 * rand(N,1)) );
 
% ----------------Plot and display 3D workspace----------------------
for n = 1:N
    theta = [theta1(n),theta2(n),theta3(n),theta4(n),theta5(n)];
    workspace = fivelink.fkine(theta);
    plot3(workspace.t(1),workspace.t(2),workspace.t(3),'b.','markersize',1);  
    hold on;
    % Save reachable coordinates
    x(n) = workspace.t(1);
    y(n) = workspace.t(2);
    z(n) = workspace.t(3);
end

%---------Display the control panel to simulate forward kinematics-------
fivelink.plot(theta); 
fivelink.teach();

%-----------------------------------------------------------
% The following codes are used to plot 3D and 2D workspaces
% Remove the comment markers before using it
%----------------------------------------------------------

% figure(1);
% subplot(2,2,1);
% plot3(x,y,z,'b.','markersize',1);  
% set(gca, 'FontSize', fontsize);
% title(['(a) 3D Workspace of Designed Robot'],"FontSize", titlesize);
% xlabel('X (mm)',"FontSize", fontsize);
% ylabel('Y (mm)',"FontSize", fontsize);
% zlabel('Z (mm)',"FontSize", fontsize);
% grid on;
%   
% subplot(2,2,2);
% plot(x,z,'b.','markersize',1);
% set(gca, 'FontSize', fontsize);
% title(['(b) X versus Z Coordinate'],"FontSize", titlesize);
% xlabel('X Coordinate (mm)',"FontSize", fontsize);
% ylabel('Z Coordinate (mm)',"FontSize", fontsize);
% ylim([-400,1000])
% grid on;
%     
% subplot(2,2,3);
% plot(y,z,'b.','markersize',1);
% set(gca, 'FontSize', fontsize);
% title(['(c) Y versus Z Coordinate'],"FontSize", titlesize);
% xlabel('Y Coordinate (mm)',"FontSize", fontsize);
% ylabel('Z Coordinate (mm)',"FontSize", fontsize);
% ylim([-400,1000])
% grid on;
% 
% subplot(2,2,4);
% plot(x,y,'b.','markersize',1);
% set(gca, 'FontSize', fontsize);
% title(['(d) X versus Y Coordinate'],"FontSize", titlesize);
% xlabel('X Coordinate (mm)',"FontSize", fontsize);
% ylabel('Y Coordinate (mm)',"FontSize", fontsize);
% grid on;
