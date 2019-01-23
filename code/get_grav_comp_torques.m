function [ torque ] = get_grav_comp_torques(theta)
        % get_grav_comp_torques
        %
        %   Calculates the joint torques required to cancel out effects due to
        %   gravity.
        
        %% DH Parameters definite a new robot with 8 frames
        % Make sure every joint has an origin
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
        joint_angle = [0;0;0;-pi/2;0;-pi/2;0;0]; %theta
        
        dh_parameter= [link_length,link_twist,link_offset,joint_angle];
        
        %% create the robot
        robot = Robot3D(dh_parameter);
        
        %% Adjust theta for 8 frames
        theta = [theta(1);0;theta(2);theta(3);0;theta(4);0;theta(5)];
        
        %% Get information about the robot:
        % Extract mass of the links, joint, and end effector [kg]
        m_joint_2 = 0.4;
        m_joint_3 = 0.4;
        m_joint_4 = 0.45;
        m_joint_5 = 0.45;
        m_end_effector = 0.10;

        %% Get the jacobian
        J = robot.jacobians(theta);
        
        %% Calculate forces and torques on Joint 2,3,4,5
        desiredForce_m_joint_2 = [0;0;9.8*m_joint_2;0;0;0];
        torque_joint_2 = transpose(J(:,:,2))*desiredForce_m_joint_2;

        desiredForce_m_joint_3 = [0;0;9.8*m_joint_3;0;0;0];
        torque_joint_3 = transpose(J(:,:,4))*desiredForce_m_joint_3;

        desiredForce_m_joint_4 = [0;0;9.8*m_joint_4;0;0;0];
        torque_joint_4 = transpose(J(:,:,5))*desiredForce_m_joint_4;

        desiredForce_m_joint_5 = [0;0;9.8*m_joint_5;0;0;0];
        torque_joint_5 = transpose(J(:,:,7))*desiredForce_m_joint_5;

        desiredForce_m_end_effector = [0;0;9.8*m_end_effector;0;0;0];
        torque_m_end_effector= transpose(J(:,:,8))*desiredForce_m_end_effector;
        
        torque = torque_joint_2 + torque_joint_3 + torque_joint_4 + ...
            torque_joint_5 + torque_m_end_effector;

        %% Pack into a more readable format. DO NOT CHANGE!
        torque = cat(1, 0, torque(2,:), torque(4,:), torque(5,:), torque(7,:));
end
