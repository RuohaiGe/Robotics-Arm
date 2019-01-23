%This file is used to test DH

%% Read CSV file
Joint_Angle = csvread("waypoints_joints.csv");
XYZ_waypoint = csvread("waypoints.csv");

%% Create a Robot
l1 = 116.23/1000;
l2 = 91.05/1000;
l3 = 327.76/1000;
l4 = 91.05/1000;
l5 = 94.05/1000;
l6 = 279.5/1000;
l7 = 57.40/1000;
l8 = 266.7/1000;
link_length = [0;0;l3;0;0;l6;0;l8];%a
link_twist = [pi/2;0;pi;-pi/2;0;pi/2;0;0];%alpha 
link_offset = [l1;l2;0;l4;l5;0;l7;0]; %d
joint_angle = [0;0;0;-pi/2;0;pi/2;0;0]; %theta
dh_parameter= [link_length,link_twist,link_offset,joint_angle];
robot = Robot3D(dh_parameter); 

%% Calculate End-effector position
n = size(Joint_Angle,1);
pose = zeros(n,3);
for index = 1:n
  theta = transpose(Joint_Angle(index,:));
  theta = [theta(1,:);0;theta(2,:);theta(3,:);0;theta(4,:);0;theta(5,:)];
  ee = robot.end_effector(theta);
  pose(index,:) = [ee(1),ee(2),ee(3)];
end

%% Plot and Compare
plot3(pose(:,1),pose(:,2),pose(:,3),'r', 'LineWidth', 1) 
hold on
plot3(XYZ_waypoint(:,1),XYZ_waypoint(:,2),XYZ_waypoint(:,3),'b', 'LineWidth', 1)
xlabel('x')
ylabel('y')
zlabel('z')