classdef rosClassHandle_UR5e
    
    properties      

        % Subscribers
        joint_state_sub;

        % Services     
        res_client;        
        get_models_client;
        get_models_state_client;

        % Actions        
        point;        
        TimeFromStart;
        trajPts;
        trajPtsVar;

        grip_action_client;
        pick_traj_act_client;
        
        % Robot
        UR5eROBOT;
        initialRobotJConfig;

        % IK
        ik;
        ik_weights;

        % TF
        tftree;
        tf_listening_time;        

        % Images
        %ds;        
        rgb_sub;
        pt_cloud_sub;

        % Yolo NN
        general_detector;

    end

    methods 
        function r = rosClassHandle_UR5e
            r.joint_state_sub         = rossubscriber("/joint_states");

            %ds                      = rossubscriber("/camera/depth/points",'DataFormat','struct');
    
            % Services
            r.res_client              = rossvcclient('/gazebo/reset_world', 'std_srvs/Empty', 'DataFormat', 'struct');                                   
                        
            r.get_models_client       = rossvcclient('/gazebo/get_world_properties', 'DataFormat','struct');            
            r.get_models_state_client = rossvcclient('/gazebo/get_model_state','DataFormat','struct');                                                               
            
            % Robot
            r.UR5eROBOT               = loadrobot("universalUR5e", "DataFormat", "row");
            r.UR5eROBOT               = urdfAdjustment(r.UR5eROBOT,"UR5e",0);
            r.initialRobotJConfig     = [0,0,0,0,0,0];

            % IKs
            r.ik                      = inverseKinematics("RigidBodyTree",r.UR5eROBOT);
            r.ik_weights              = [0.25, 0.25, 0.25, 0.1, 0.1, 0.1];  

            % TF
            r.tftree                  = rostf("DataFormat", "struct"); % **Complete Code**
            r.tf_listening_time       = 10;
        end

    end

end