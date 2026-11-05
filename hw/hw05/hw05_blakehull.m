%% Data Collection
% Use this section to collect data for your skills
function hw05_blakehull(ctraj1,ctraj2)
% A)Update Names
% 1. Replace function name with your firstlast name.
%    For example, if your name is Juan Rojas do: Rout = hw03_juanrojas(Rin)
% 2. Save this function with your name, i.e.: hwXX_juanrojas.m
%
% B) This top function contains: one data collection section and 3
% functions. These functions need to be called by you. 

    %% 1.A Top Pick
    function top_pick_ik(ctraj1)
    %--------------------------------------------------------------------------
    % Top-Pick Skill
    % Recreate the trajectory from HW04b - FW Kins for your top-pick, but
    % this time by using the poses output by your function. Use ctraj and
    % inverse kinematics to recreate the skill.
    %
    % Inputs:
    % ctraj1: [doulbe]  The 4x4x5 matrix composed of 5 4x4 matrices corresponding 
    %                   to the poses of the 5 way-points.
    %----------------------------------------------------------------------
        % BEGIN_YOUR_CODE 
        panda = loadrobot("frankaEmikaPanda", DataFormat = "row");
        panda_ik = inverseKinematics(RigidBodyTree = panda);
        q_guess = [0.9859   -0.2546   -1.8848   -1.6053   -0.0833    1.8076    1.0172    0.0318  0.0400];
        
        for wp = 1 : size(ctraj1, 3)

            if (wp + 1) <= size(ctraj1, 3)
                TA = ctraj1(:, :, wp);
                TB = ctraj1(:, :, wp+1);
                traj1 = ctraj(TA, TB, 50);
                q_top = zeros(size(traj1, 3), 9);
                
                for i = 1 : size(traj1, 3)
                    T = traj1(:, :, i);
                    q_step = panda_ik("panda_hand", T, ones(1,6), q_guess);
                    q_top(i, :) = q_step;
                    q_guess = q_step;
                end

                r = rateControl(10);
                for i = 1 : size(q_top, 1)
                    q_disp = q_top(i, :);
                    show(panda, q_disp, FastUpdate=true, PreservePlot=false);
                    waitfor(r);
                end
            end
        end
        % END_YOUR_CODE
    end
    
    %% 1.B Side-Pick
    function side_pick_ik(ctraj2)
    %--------------------------------------------------------------------------
    % Side-Pick Skill
    % Recreate the trajectory from HW04b - FW Kins for your side-pick, but
    % this time by using the poses output by your function. Use ctraj and
    % inverse kinematics to recreate the skill.
    %
    % Inputs:
    % ctraj1: [doulbe]  The 4x4x5 matrix composed of 5 4x4 matrices corresponding 
    %                   to the poses of the 5 way-points.
    %--------------------------------------------------------------------------
        % BEGIN_YOUR_CODE 
        panda = loadrobot("frankaEmikaPanda", DataFormat = "row");
        panda_ik = inverseKinematics(RigidBodyTree = panda);
        q_guess = [0.9859   -0.2546   -1.8848   -1.6053   -0.0833    1.8076    1.0172    0.0318  0.0400];
        
        for wp = 1 : size(ctraj2, 3)

            if (wp + 1) <= size(ctraj2, 3)
                TA = ctraj2(:, :, wp);
                TB = ctraj2(:, :, wp+1);
                traj2 = ctraj(TA, TB, 50);
                q_side = zeros(size(traj2, 3), 9);
                
                for i = 1 : size(traj2, 3)
                    T = traj2(:, :, i);
                    q_step = panda_ik("panda_hand", T, ones(1,6), q_guess);
                    q_side(i, :) = q_step;
                    q_guess = q_step;
                end

                r = rateControl(10);
                for i = 1 : size(q_side, 1)
                    q_disp = q_side(i, :);
                    show(panda, q_disp, FastUpdate=true, PreservePlot=false);
                    waitfor(r);
                end
            end
        end
        % END_YOUR_CODE
    end
  
    %% 2 Follower
    function follower(T1,T2)
    %--------------------------------------------------------------------------
    % Create a follower skill where the panda robot moves in a straight line and
    % the orientation never changes.
    %
    % Inputs:
    % T1 - (double) a 3D pose to start
    % T2 - (double) a 3D pose to end
    %--------------------------------------------------------------------------
        % BEGIN_YOUR_CODE 
        T1 = T1.tform;
        T2 = T2.tform;
        
        panda = loadrobot("frankaEmikaPanda", DataFormat = "row");
        q_guess = [0.9859   -0.2546   -1.8848   -1.6053   -0.0833    1.8076    1.0172    0.0318  0.0400];
        panda_ik = inverseKinematics(RigidBodyTree = panda);
        steps = 50;
        traj3 = ctraj(T1, T2, steps);
        
        q = [];
        
        for i = 1 : size(traj3, 3)
            T = traj3(:, :, i);
            q_step = panda_ik("panda_hand", T, ones(1,6), q_guess);
            q(end + 1, :) = q_step;
            q_guess = q_step;
        end

        r = rateControl(10);
        for i = 1 : size(q, 1)
            q_disp = q(i, :);
            show(panda, q_disp, FastUpdate=true, PreservePlot=false);
            waitfor(r);
        end
        % END_YOUR_CODE
    end
    %% Data collection
    [top_down_pick_joints, side_pick_joints] = hw04_blakehull;

    %% Call Functions -- you will define them further down
    top_pick_ik(top_down_pick_joints);
    side_pick_ik(side_pick_joints);
    
    % Fill the following 
    %T1 position: [0.25, -0.25, 0.25], fingers parallel to world x-axis
    T1 = se3([0.25 -0.25 0.25],'trvec');

    %T2 position: [0.25, +0.25, 0.25], fingers parallel to world x-axis
    T2 = se3([0.25  0.25 0.25],'trvec');

    % Move them and record
    follower(T1,T2)

end