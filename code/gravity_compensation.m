function [] = gravity_compensation()

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

%To load the gains_struct from file
gains = load('robotA_gains.mat');  %contains gains_struct
gains_struct = gains.gains_struct;
gains_struct.positionKp = [3.3 6 6 4 4];         
gains_struct.positionKi = [0.005 0 0 0 0];     
gains_struct.positionKd = [0.005 0 0 0 0]; 
% display(gains_struct) will show you other fields you can modify
robotHardware.set('gains', gains_struct); % note this is the set function, not send

%% Setup reusable structures to reduce memory use in loop
cmd = CommandStruct();
tmpFbk = robotHardware.getNextFeedback();

frequency = 100;
%% Test certain path
Joint_Angle = load('trajectory.mat');
Joint_Angle = Joint_Angle.trajectory;
num_waypoints = size(Joint_Angle,2);
times = 1/10:1/10:num_waypoints/10;
trajectory_splined = trajectory_spline(Joint_Angle, times, frequency);

theta = [0.3704;0.8493;0.9380;-0.0691;0.3310];

while true
     
    % Read Robot feedback
    fbk = robotHardware.getNextFeedback(tmpFbk);
    fbk.position
    theta =  [0.4321;0.0533;0.0580;0.3189;-0.0151]';
    torque = get_grav_comp_torques(theta);
    cmd.position = theta;
    % Note that first torque is always 0 
    % Note that we need to account for the variable ee additional force
    cmd.torque = [torque(1),torque(2),torque(3),torque(4),torque(5)]; 
    
    % Send command to robot; limit velocity to damp out fast motions.
    % cmd.velocity = [0,0];
    robotHardware.set(cmd);

    % Wait a little bit; here we'll cap command rates to 100Hz.
    pause(0.1);
    
end


% 
