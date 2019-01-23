frequency = 100;

waypoints = csvread("straight.csv");% waypoints of trajectories to be defined by students. Think about important points
first_homeposition_theta = [0.3704;0.8493;0.9380;-0.0691;0.3310];
waypoint = [0.4758;0.7218;0.8167;-0.0135;0.4382];
waypoints = waypoints - [0.04,0,0.02];
[trajectory_splined_test,trajectory_splined]
% initial_theta = robot.numerical_IK(waypoints(1,:)', first_homeposition_theta,...
%      [first_homeposition(1);first_homeposition(2);first_homeposition(3)], 1);
initial_theta = waypoint;
trajectory = [];
trajectory_splined_test = trajectory_spline([first_homeposition_theta,waypoint], [1,2], frequency);
%trajectory = [initial_theta];

%% Time
%% Run the trajectory generation and command trajectory
for index = 2:size(waypoints,1)
    goal_position = transpose(waypoints(index,:));
    goal_position_prev = transpose(waypoints(index-1,:));
    goal_angles = robot.numerical_IK(goal_position, initial_theta, goal_position_prev, 1);
    trajectory = [trajectory,goal_angles];
    initial_theta = goal_angles;
end
trajectory;
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
a = [trajectory_splined_test,trajectory_splined]
save('trajectory','a');
