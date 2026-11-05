%% Data Collection
% Use this section to collect data for your skills
function [tdp_ctraj, sp_ctraj,jLims]=hw04_blakehull()
% A)Update Names
% 1. Replace function name with your firstlast name.
%    For example, if your name is Juan Rojas do: Rout = hw03_juanrojas(Rin)
% 2. Save this function with your name, i.e.: hwXX_juanrojas.m
%
% B) This top function contains: one data collection section and 3
% functions. These functions need to be called by you. 

    %% 1.A Top Pick
    function tdp_ctraj = top_pick(top_down_pick_joints)
    %--------------------------------------------------------------------------
    % Top-Pick Skill
    % Produce a skill that is able to pick an imaginary object from the top.
    % This skill should have four trajectory points:
    % 0. Start: robot should start from some ready position
    % 1. Pre-pick: approach object location, but stop some small distance (10cm) above
    % 2. Pick: Perform top-down motion to approach the object (we will not close
    % the fingers here)
    % 3. Pre-Place: approach place location, but stop some small distance (10cm) above
    % 4. Place: Perform top-down motion to place object. 
    %
    % Inputs:
    % top_down_pick_joints: [double] A 5x9 matrix consisting of 5 joint angle
    % configurations to achieve a top-pick skill with the panda robot.
    %
    % Outputs:
    % tdp_ctraj: [double] A 4x4x5 matrix composed of 5 4x4 matrices corresponding 
    %             to the forward kinematics at each of the 5 way-points.
    %----------------------------------------------------------------------
    % BEGIN_YOUR_CODE 
        tdp_ctraj = [];
        for p = 1:size(top_down_pick_joints, 1)
            q = top_down_pick_joints(p, :);
            wp = robot.getTransform(q, "panda_hand"); % REVIEW USE OF fkine. Try getTransform
            tdp_ctraj(:,:,p) = wp;
        end
    % END_YOUR_CODE
    end
    
    %% 1.B Side-Pick
    function sp_ctraj = side_pick(joint_path)
    %--------------------------------------------------------------------------
    % Top-Pick Skill
    % (B) Side-Pick Skil
    % Same as above but coming from the a side of the robot (either +y or -y
    % ok).
    %
    % Inputs:
    % side_pick_joints: [double] A 5x9 matrix consisting of 5 joint angle
    % configurations to achieve a side-pick skill with the panda robot.
    %
    % Outputs:
    % sp_ctraj: [double] A 4x4x5 matrix composed of 5 4x4 matrices corresponding 
    %             to the forward kinematics at each of the 5 way-points.
    %--------------------------------------------------------------------------
    % BEGIN_YOUR_CODE 
        sp_ctraj = [];
        for p = 1:size(joint_path, 1)
                q = joint_path(p, :);
                wp = robot.getTransform(q, "panda_hand"); % REVIEW USE OF fkine. Try getTransform
                sp_ctraj(:,:,p) = wp;
        end
    % END_YOUR_CODE
    end
    
    %--------------------------------------------------------------------------
    % 2. Joint Limits
    % Identify the joint limits of the robot.
    %----------------------------------------------------------------------
    
    %% 2. Joint Limits
    function jointAngles = getRobotJointLimits(robot)
    %--------------------------------------------------------------------------
    % Returns jointAngles structure that is indexed by the name of the joint 
    % and contains a 2x1 array of joint limits.
    %
    % Only consider joints 2-7 in the panda robot. Each joint will have a 
    % lower limit and an upper limit.
    %
    % When the name of the joint is provided as a field to the structure:
    %   jointAngles.(jointName) the structure returns the lower and upper
    %   limits for that joint.
    % 
    % Input:    
    %       robot [rigidBodyTree]: robot model 
    % Output:   
    %       jointAngles [struct]: field name will be the relevant joint
    %       name with each field having a 1x2 array of joint limits.
    %--------------------------------------------------------------------------
    % BEGIN_YOUR_CODE 
        jointAngles = struct();
        for j = 2:7
            jointAngles.(robot.Bodies{j}.Joint.Name) = robot.Bodies{j}.Joint.PositionLimits;
        end
    % END_YOUR_CODE
    end
  

    %% Data structures. 
    % To be filled by creating an interactive rigid body tree
    % irbt=interactiveRigidBodyTree(robot,Configuration) 
    % 
    % And using irbt.Configuration to store a starting joint configuration and 
    % 4 more points to complete a top-down pick as described in the top_pick()
    % docstring and another star/4-way points to complete a side-pick as
    % described in side_pick().
    %--------------------------------------------------------------------------

    robot = loadrobot("frankaEmikaPanda",DataFormat="row");
    
    % Top-Down Pick
    top_down_pick_joints=zeros(5,9);
    % BEGIN_YOUR_CODE 
    top_down_pick_joints(1, :) = [-0.4313, -0.7213, 0.2062, -2.0009, 0.1921, 1.2984, 0.5777, 0, 0.0049];
    top_down_pick_joints(2, :) = [-0.4849, -0.6197, 0.0115, -1.9808, 0.0569, 1.3529, 0.3396, 0, 0];
    top_down_pick_joints(3, :) = [-0.5038, -0.6549, -0.0347, -2.4807, 0.0263, 1.8119, 0.2600, 0, 0.0400];
    top_down_pick_joints(4, :) = [-0.3493, -0.8300, 0.4151, -2.2119, 0.3524, 1.4525, 0.7874, 0, 0.0400];
    top_down_pick_joints(5, :) = [-0.3201, -0.8608, 0.5257, -2.4697, 0.4385, 1.6800, 0.8010, 0, 0.0275];
    % END_YOUR_CODE
    
    % Side-Pick
    side_pick_joints=zeros(5,9);
    % BEGIN_YOUR_CODE 
    side_pick_joints(1, :) = [-0.4313, -0.7213, 0.2062, -2.0009, 0.1921, 1.2984, 0.5777, 0, 0.0049];
    side_pick_joints(2, :) = [-0.2294, 0.1248, -0.2437, -2.7274, 1.1614, 1.3803, -0.5405, 0, 0];
    side_pick_joints(3, :) = [-0.0728, 0.1465, -0.0780, -2.7777, 1.4431, 1.4764, -0.5793, 0, 0.0022];
    side_pick_joints(4, :) = [-0.6684, 0.7102, -0.4267, -1.8612, 0.7403, 0.8815, -0.6280, 0, 0];
    side_pick_joints(5, :) = [-0.5406, 0.6600, -0.3775, -2.1078, 0.8184, 1.0689, -0.6764, 0, 0.0400];
    % END_YOUR_CODE
    
    %% Call Functions -- you will define them further down
    tdp_ctraj = top_pick(top_down_pick_joints);
    sp_ctraj = side_pick(side_pick_joints);
    jLims = getRobotJointLimits(robot);
end