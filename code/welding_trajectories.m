function [] = welding_trajectories()
% feeding_trajectories

%% Clear out old information to reduce problems with stale modules
HebiLookup.setLookupAddresses('*');
HebiLookup.clearModuleList();
HebiLookup.clearGroups();
pause(3);

%% Connect to physical robot
robotHardware = HebiLookup.newGroupFromNames('Robot A',{'J1','J2', 'J3', 'J4', 'J5'});
robotHardware.setCommandLifetime(2);

warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
disp('');
input('Once ready, press "enter" to continue...','s');
gains = load('robotA_gains.mat');
gains_struct = gains.gains_struct;
gains_struct.positionKp = [3.3 5.5 3 5 3];         
gains_struct.positionKi = [0.005 0 0 0 0];     
gains_struct.positionKd = [0.005 0.01 0 0 0]; 


robotHardware.set('gains', gains_struct);

%% Start logging
currentDir = fileparts(mfilename('fullpath'));
logFile = robotHardware.startLog('file', fullfile(currentDir, 'robot_data'));

%% command frequency, in Hz
frequency = 100;

%% DH Parameters definition to be defined by student 
l1 = 116.23/1000;
l2 = 91.05/1000;
l3 = 327.76/1000;
l4 = 91.05/1000;
l5 = 94.05/1000;
l6 = 279.5/1000;
l7 = 57.40/1000;
l8 = 266.7/1000;
link_length = [0;l3;0;l6;l8];%a

link_twist = [pi/2;pi;-pi/2;pi/2;0];%alpha 
link_offset = [l1;l2;l4;l5;l7]; %d
joint_angle = [0;0;-pi/2;-pi/2;0]; %theta
dh_parameter= [link_length,link_twist,link_offset,joint_angle];
robot = Robot3D(dh_parameter);
%% Move the robot to Home Position
disp('');
input('Put the robot in a safe position...','s');
% trajectory = homePositioning(robotHardware, frequency);

%% Load and Define waypoints and time stamps for your trajectory
waypoints = csvread("straight.csv");% waypoints of trajectories to be defined by students. Think about important points
first_homeposition_theta = [0.3704;0.8493;0.9380;-0.0691;0.3310];
waypoint = [0.4758;0.7218;0.8167;-0.0135;0.4382];
% waypoints(:,3) = waypoints(:,3)+ [ones(50,1)*-0.15; ones(30,1)*0.05; ones(21,1)*0.1];
waypoints = waypoints - [0.04,0,0.016];
initial_theta = waypoint;
trajectory = [];
trajectory_splined_test = trajectory_spline([first_homeposition_theta,waypoint], [1,2], frequency);

%% Time
%% Run the trajectory generation and command trajectory
for index = 2:size(waypoints,1)
    goal_position = transpose(waypoints(index,:));
    goal_position_prev = transpose(waypoints(index-1,:));
    goal_angles = robot.numerical_IK(goal_position, initial_theta, goal_position_prev);
    trajectory = [trajectory,goal_angles];
    initial_theta = goal_angles;
end
trajectory
n = size(trajectory,2);
pose = zeros(n,4);

for index = 1:n
  theta = trajectory(:,index);
  theta = [theta(1,:);theta(2,:);theta(3,:);theta(4,:);theta(5,:)];
  ee = robot.end_effector(theta);
  pose(index,:) = [ee(1),ee(2),ee(3),(ee(6))/pi*180];
end

pose

%% Plot and Compare
plot3(pose(:,1),pose(:,2),pose(:,3),'r', 'LineWidth', 1) 
hold on
plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3),'b', 'LineWidth', 1)
xlabel('x')
ylabel('y')
zlabel('z')
hold off

times = 1/10:1/10:size(trajectory,2)/10; 
trajectory_splined = trajectory_spline(trajectory, times, frequency);

% %% Define Home position for start and finish
% % command cmd
homePositioning(robotHardware, frequency, first_homeposition_theta);

%trajectory_splined = trajectory_spline([start,trajectory], times, frequency);
command_trajectory(robotHardware, [trajectory_splined_test,trajectory_splined], frequency);
 
%% Stop logging, and plot results
robotHardware.stopLog();

hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'robot_data.hebilog'));

% Plot angle data
figure();
subplot(3,1,1);
plot(hebilog.time, hebilog.positionCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.position, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint positions during trajectory');
xlabel('t');
ylabel('\theta');
subplot(3,1,2);
plot(hebilog.time, hebilog.velocityCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.velocity, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint velocities during trajectory');
xlabel('t');
ylabel('joint velocities');
subplot(3,1,3);
plot(hebilog.time, hebilog.torque, 'r--', 'LineWidth', 1)
title('Plot of joint torques during trajectory');
xlabel('t');
ylabel('\tau');

end
