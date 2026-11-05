function top_down_pick_joints=load_top_pick_joint_data   
% Load your 5 joint configurations acquired from via the
% interactiveRigidBodyTree() setup.
% Inputs:
%           None
% Outputs:
%           top_down_pick_joints: [double] a 5x9 matrix of robot joint
%           angles.
%--------------------------------------------------------------------------
% BEGIN_YOUR_CODE 
%[top_down_pick_joints] = hw04_blakehull()
% top_down_pick_joints=cell();

top_down_pick_joints(1, :) = [-0.4313, -0.7213, 0.2062, -2.0009, 0.1921, 1.2984, 0.5777, 0, 0.0049];
top_down_pick_joints(2, :) = [-0.4849, -0.6197, 0.0115, -1.9808, 0.0569, 1.3529, 0.3396, 0, 0];
top_down_pick_joints(3, :) = [-0.5038, -0.6549, -0.0347, -2.4807, 0.0263, 1.8119, 0.2600, 0, 0.0400];
top_down_pick_joints(4, :) = [-0.3493, -0.8300, 0.4151, -2.2119, 0.3524, 1.4525, 0.7874, 0, 0.0400];
top_down_pick_joints(5, :) = [-0.3201, -0.8608, 0.5257, -2.4697, 0.4385, 1.6800, 0.8010, 0, 0.0275];
% END_YOUR_CODE
end