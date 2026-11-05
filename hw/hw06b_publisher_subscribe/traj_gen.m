function traj_gen(ctraj_pts, robot_handle, robot_ik_num, q_guess)
%--------------------------------------------------------------------------
% Top-Pick Skill
% Create a trajectory based on ctraj waypoints that can be expanded via 
% ctraj and inverse kinematics.
%
% Inputs:
% ctraj1: [doulbe]  The 4x4x5 matrix composed of 5 4x4 matrices corresponding 
%                   to the poses of the 5 way-points.
%----------------------------------------------------------------------
    % BEGIN_YOUR_CODE 
    %% Initialize homogeneous transform matrix
    %% Generate cartesian trajectories across way points:
    %% Generate the joint angle configurations for all of these steps. Also publish and display them.
    
    %panda = loadrobot("frankaEmikaPanda", DataFormat = "row");
    %panda_ik = inverseKinematics(RigidBodyTree = panda);
    %q_guess = [0.9859   -0.2546   -1.8848   -1.6053   -0.0833    1.8076    1.0172    0.0318  0.0400];

    for wp = 1 : size(ctraj_pts, 3) % wp is index counting from 1 to the size of ctraj_pts 3rd dim

        if (wp + 1) <= size(ctraj_pts, 3)
            TA = ctraj_pts(:, :, wp);
            TB = ctraj_pts(:, :, wp+1);
            traj1 = ctraj(TA, TB, 50);
            % q_top = zeros(size(traj1, 3), 9);
            q_top = cell(size(traj1, 3), 1); % move outside loop?
            
            for i = 1 : size(traj1, 3)
                T = traj1(:, :, i);
                q_step = robot_ik_num(robot_handle.end_effector, T, robot_handle.ik_weights, q_guess); % "panda_hand" may cause problems
                % q_step = robot_ik_num;
                q_top{i} = q_step;
                q_guess = q_step;
            end

            r = rateControl(10);
            joint_message = rosmessage(robot_handle.joint_state_pub); % move outside loop?
            for i = 1 : size(q_top, 1)
                q_disp = q_top{i};
                show(robot_handle.robot, q_disp, FastUpdate=true, PreservePlot=false);
                joint_message.Position = q_disp;
                send(robot_handle.joint_state_pub, joint_message);
                %pub_matlab_joints([], joint_message);
                waitfor(r);
            end
        end
    end
    % END_YOUR_CODE
end
