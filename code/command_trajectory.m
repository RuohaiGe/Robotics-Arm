% Define a simple function to actually send a series of points to the
% robot, where the 'trajectory' is a matrix of columns of joint angle
% commands to be sent to 'robot' at approximately 'frequency'.
% Note this also commands velocities, although you can choose to only command
% positions if desired, or to add torques to help compensate for gravity.

function [] = command_trajectory(robotHardware, trajectory, frequency)
  %% Setup reusable structures to reduce memory use in loop
  cmd = CommandStruct();
  
  %% calculate torque for each waypoints
  torque = [];
  for index_torque = 1:size(trajectory,2)
    goal_angles = trajectory(:,index_torque);
    torque_cur = get_grav_comp_torques(goal_angles);
    torque = [torque,torque_cur];
  end

  %% Compute the velocity numerically
  trajectory_vel = diff(trajectory, 1, 2);
  save('trajectory','trajectory');
  %% Command the trajectory
  for i = 1:(size(trajectory, 2) - 1)
    % Send command to the robot (the transposes on the trajectory
    % points turns column into row vector for commands).
    cmd.position = trajectory(:,i)';
    cmd.velocity = trajectory_vel(:,i)' * frequency;
    cmd.torque = torque(:,i)';
    robotHardware.set(cmd);
    % Wait a little bit to send at ~100Hz.
    pause(1 / frequency);
  end

  %% Send the last point, with a goal of zero velocity.
  cmd.position = trajectory(:,end)';
  cmd.velocity = zeros(1, size(trajectory, 1));
  robotHardware.set(cmd);
end