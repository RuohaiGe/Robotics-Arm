classdef Robot3D
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'immutable')
        dof
        link_length %a
        link_twist %alpha
        link_offset %d
        joint_angle %theta
    end
    
    methods
        %% Constructor: Makes a brand new robot with the specified parameters.
        function robot = Robot3D(dh_parameter)
            link_length = dh_parameter(:,1);
            link_twist = dh_parameter(:,2);
            link_offset = dh_parameter(:,3);
            joint_angle = dh_parameter(:,4);
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(link_length, 2) ~= 1
               error('Invalid link_length: Should be a column vector, is %dx%d.', size(link_length, 1), size(link_length, 2));
            end
            
            if size(link_twist, 2) ~= 1
               error('Invalid link_twist: Should be a column vector, is %dx%d.', size(link_twist, 1), size(link_twist, 2));
            end
            
            if size(link_offset, 2) ~= 1
               error('Invalid link_offset: Should be a column vector, is %dx%d.', size(link_offset, 1), size(link_offset, 2));
            end
            
            if size(joint_angle, 2) ~= 1
               error('Invalid joint_angle: Should be a column vector, is %dx%d.', size(joint_angle, 1), size(joint_angle, 2));
            end
            
            robot.dof = size(link_length, 1);
                    
            if size(link_twist, 1) ~= robot.dof
                error('Invalid number of link_twists: should match number of link lengths.');
            end
            
            if size(link_offset, 1) ~= robot.dof
                error('Invalid number of link_twist: should match number of link lengths.');
            end
            
            if size(joint_angle, 1) ~= robot.dof
                error('Invalid number of joint_angle: should match number of link lengths.');
            end
            
            robot.link_length = link_length;
            robot.link_twist = link_twist;
            robot.link_offset = link_offset;
            robot.joint_angle = joint_angle;  
        end
       
        %% Returns the forward kinematic map for each frame, one for the base of
        % each link, and one for the end effector. Link i is given by
        % frames(:,:,i), and the end effector frame is frames(:,:,end).
        function frames = forward_kinematics(robot, thetas)
            if size(thetas, 2) ~= 1
                error('Expecting a column vector of joint angles.');
            end
            
            if size(thetas, 1) ~= robot.dof
                error('Invalid number of joints: %d found, expecting %d', size(thetas, 1), robot.dof);
            end
            
            % Allocate a variable containing the transforms from each frame
            % to the base frame.
            frames = zeros(4,4,robot.dof);
            n = robot.dof;
            % The transform from the base of link 'i' to the base frame (H^0_i)
            % is given by the 4x4 matrix frames(:,:,i).
            
            % Get the first frame
            alpha = robot.link_twist(1);
            a = robot.link_length(1);
            d = robot.link_offset(1);
            theta = thetas(1)+ robot.joint_angle(1);
            frames(:,:,1) = [cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta);
                sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta);
                0,sin(alpha),cos(alpha),d;
                0,0,0,1];
            % Calculate the rest using the first frame
            for frame_index = 2:n
                alpha = robot.link_twist(frame_index);
                a = robot.link_length(frame_index);
                d = robot.link_offset(frame_index);
                theta = thetas(frame_index)+ robot.joint_angle(frame_index);
                frames(:,:,frame_index) = frames(:,:,frame_index-1) * ...
                 [cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta);
                  sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta);
                  0,sin(alpha),cos(alpha),d;
                  0,0,0,1];
            end
        end
        
        function goal_angles = numerical_IK(robot, goal_position, initial_theta, goal_position_prev)
            %This function takes a "Robot" object (containing a end_effector
            %function), a column vector goal position (in this case, just x, y, z position)
            %and an initial set of joint angles (in column vector form).
            
            %It should return a column vector of destination angles.
            % The first task is to generate an "error function", which describes how
            % far from the goal any given set of joint angles is.  In past assignments,
            % we just used the square of the Euclidean distance; this is what we will do
            % below.
            function err = my_error_function(theta)   
              actual_pos = robot.end_effector(theta);
              actual_position = actual_pos(1:3);
              err = (goal_position - actual_position).^2;
              err = sum(err);
              
              %fourth degree of freedom
              frames = robot.forward_kinematics(theta);
              frame = frames(:,:,end);
              
              % projection on the board 
              x_y = frame(1,2);
              z_y = frame(3,2);
              
              % actual position
              dx = goal_position(1)-goal_position_prev(1);
              dz = goal_position(3)-goal_position_prev(3);
              
              
              diff = abs(atan2(dz,dx) - atan2(z_y, x_y));
              if (diff > pi/2)
                  diff = pi - diff;
              end
              
              % update error
              err = err + 0.01*(diff)^2;
            end
           
            % Actually run the optimization to generate the angles to get us (close) to
            % the goal.
            % Set joint limit for better performance
            lb = [0,pi/6,-pi,-pi,-pi];
            ub = [pi,pi,pi,pi,pi/4];
            
            %Final goal_angles
            goal_angles = fmincon( @ my_error_function,initial_theta,[],[],[],[],lb,ub);
            goal_angles = wrapToPi(goal_angles);
        end

%% Jacobian
        function jacobians = jacobians(robot, thetas)
            % Returns the SE(3) Jacobian for each frame (as defined in the forward
            % kinematics map). Note that 'thetas' should be a column vector.

            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
               error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end

            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            jacobians = zeros(6,robot.dof,robot.dof); 
            frames = robot.forward_kinematics(thetas);
            for frame_index = 1 : robot.dof
                end_frame = frames(:,:,frame_index);
                end_frame = end_frame(:,4);
                o_n= [end_frame(1,:); end_frame(2,:); end_frame(3,:)]; %[x;y;z]
                           
                z = zeros(3,1,robot.dof); %z^o_(i-1)
                o = zeros(3,1,robot.dof); %o_n - o_(i-1)
                
                for joint_index = 1 : frame_index
                    if(joint_index == 1)
                        %Frame 1
                        z(:,:,1) = [0;0;1];
                        o(:,:,1) = o_n - [0;0;0];

                        %Calculate linear and angular jacobian
                        jv1 = cross(z(:,:,1),o(:,:,1));
                        jw1 = z(:,:,1);

                        %Put it into the final jacobian
                        jacobians(:,1,frame_index) = [jv1;jw1];
                    else
                        prev_frame = frames(:,:,joint_index-1);
                        z(:,:,joint_index) = prev_frame(1:3,3);
                        o(:,:,joint_index) = o_n -  prev_frame(1:3,4);

                        %Calculate linear and angular jacobian
                        jv_frame = cross(z(:,:,joint_index),o(:,:,joint_index));
                        jw_frame = z(:,:,joint_index);

                        %Put it into the final jacobian
                        jacobians(:,joint_index,frame_index) = [jv_frame;jw_frame];
                    end
                end   
            end
        end

        
        %% Shorthand for returning the forward kinematics.
        function fk = fk(robot, thetas)
            fk = robot.forward_kinematics(thetas);
        end
       
        % Returns [x; y; theta] for the end effector given a set of joint
        % angles. 
        function ee = end_effector(robot, thetas)
            % Find the transform to the end-effector frame.
            frames = robot.fk(thetas);
            H_0_ee = frames(:,:,end);
            
            % Extract the components of the end_effector position and
            % orientation.
            x = H_0_ee(1,4);
            y = H_0_ee(2,4);
            z = H_0_ee(3,4);
            roll = atan2(H_0_ee(2, 1), H_0_ee(1, 1));
            pitch = -asin(H_0_ee(3, 1));
            yaw = atan2(H_0_ee(3, 2), H_0_ee(3, 3));
            % Pack them up nicely.
            ee = [x; y; z; roll; pitch; yaw];
        end
       
        %% Shorthand for returning the end effector position and orientation. 
        function ee = ee(robot, thetas)
            ee = robot.end_effector(thetas);
        end
    end
end