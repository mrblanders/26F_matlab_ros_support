function hw_06b_ros_pub_sub()
    %% Close ROS to ensure fresh start
    clear;
    rosshutdown;

    %% 01Start ROS via rosinit
    rosinit
    
    %% 02 Start Robot Class Handle
    r = ros_class_handle;
   
    %% 03 Load Joint Angle Data
    top_down_pick_joints = load_top_pick_joint_data;  

    %% 04 Compute ctraj goals for your top pick using panda_handa for IKs and your output for joint angle data
    ctraj_via_points = top_pick(top_down_pick_joints, r.robot,"panda_hand");

    %% 05 Create a subscriber that receives /matlab_joint_states and pubishes ros-based /joint_states
    % Ensure this is done inside ros_class_handle

    %% 06 Compute ctraj trajectory and publish to /matlab_joint_states with sensor_msgs/JointStates via traj_gen
    traj_gen(ctraj_via_points, r.robot, r.ik, r.ik_weights, r.initialRobotJConfig, r.joint_state_pub)

