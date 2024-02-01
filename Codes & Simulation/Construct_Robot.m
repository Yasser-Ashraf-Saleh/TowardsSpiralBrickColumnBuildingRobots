classdef Construct_Robot < handle
  
    
    properties
        color_space
        
        obj_mesh_ind
        obj_mesh_color_id
        obj_mesh_color
        
        cam_position
        cam_orientation        
        
        nearClip
        farClip
        
        sim_client
        UR5_target_handle
        UR5_tip_handle
        RG2_tip_handle
        simxModelproperty_not_dynamic
        cam_handle
        cam_intrinsics
        cam_depth_scale
        cam_pose
        %Proximity_sensor

        bg_color_img
        bg_depth_img
        
        
        is_testing = false
        test_preset_cases = false
        
        
        workspace_limits =  [[-1.5, -0.5]; [0.26, 0.6]; [0.21, 0.3]]; % x; y; z
        heightmap_resolution = 0.002

        vrep
        
        object_handles = [ ]
                
    end
    
    properties (Hidden)
    end
    
    methods
        
        
        function obj = Construct_Robot()            
            
            % object vrep ~ vrep software
            vrep=remApi('remoteApi');

            % close everything 
            vrep.simxFinish(-1);

            % client to vrep
            obj.sim_client =vrep.simxStart('127.0.0.1', 19997, true,true, 5000, 5);
            % (server,port,waitUntilConnected,doNotReconnectOnceDisconnected,timeOutInMs,commThreadCycleInMs) 

            if obj.sim_client == -1
                disp('Failed to connect to simulation (V-REP remote API server). Exiting.')
                exit()
            else
                disp('Connected to simulation')
                obj.restart_sim(vrep)
                obj.vrep = vrep;
            end
            [ret,obj.UR5_tip_handle]=vrep.simxGetObjectHandle(obj.sim_client,'UR5_tip', vrep.simx_opmode_blocking);
            
            % camera settings
            obj.setup_sim_camera(vrep)

        end
            
        
        function restart_sim(self, vrep)

            [sim_ret, self.UR5_target_handle] = vrep.simxGetObjectHandle(self.sim_client, 'UR5_target', vrep.simx_opmode_blocking);
            
            vrep.simxSetObjectPosition(self.sim_client, self.UR5_target_handle, -1, [-0.5, 0, 0.3], vrep.simx_opmode_blocking);
            vrep.simxStopSimulation(self.sim_client, vrep.simx_opmode_blocking);
            vrep.simxStartSimulation(self.sim_client, vrep.simx_opmode_blocking);
            pause(1)
            [sim_ret, self.RG2_tip_handle] = vrep.simxGetObjectHandle(self.sim_client, 'UR5_tip', vrep.simx_opmode_blocking);
            [sim_ret, gripper_position] = vrep.simxGetObjectPosition(self.sim_client, self.RG2_tip_handle, -1, vrep.simx_opmode_blocking);
            
            % V-REP bug requiring multiple starts and stops to restart
            while gripper_position(3) > 0.4 
                vrep.simxStopSimulation(self.sim_client, vrep.simx_opmode_blocking);
                vrep.simxStartSimulation(self.sim_client, vrep.simx_opmode_blocking);
                pause(1)
                [sim_ret, gripper_position] = vrep.simxGetObjectPosition(self.sim_client, self.RG2_tip_handle, -1, vrep.simx_opmode_blocking);
            end
        
        end
        
        
       function setup_sim_camera(self, vrep)
            
            % Get handle to camera
            [sim_ret, self.cam_handle] = vrep.simxGetObjectHandle(self.sim_client, 'Vision_sensor_persp', vrep.simx_opmode_blocking);

            % Get camera pose
            [sim_ret, cam_position] = vrep.simxGetObjectPosition(self.sim_client, self.cam_handle, -1, vrep.simx_opmode_blocking);
            [sim_ret, cam_orientation] = vrep.simxGetObjectOrientation(self.sim_client, self.cam_handle, -1, vrep.simx_opmode_blocking);
            
            self.cam_orientation = double(cam_orientation);
            self.cam_position = double(cam_position);
            
            cam_trans = eye(4);
            cam_trans(1:3, end) = cam_position';
            cam_orientation = -[cam_orientation(1), cam_orientation(2), cam_orientation(3) ];
            cam_rotm = eye(4);
            cam_rotm(1:3,1:3) = inv( euler2rotm(cam_orientation) );            
            self.cam_pose = cam_trans*cam_rotm; % Compute rigid transformation representating camera pose
            self.cam_intrinsics = [ [618.62, 0, 320];  [0, 618.62, 240] ; [0, 0, 1] ];
                        
            % camera properties (not necessary)
            [res, nearClip]= vrep.simxGetObjectFloatParameter(self.sim_client, self.cam_handle, vrep.sim_visionfloatparam_near_clipping , vrep.simx_opmode_blocking);
            [res, farClip]= vrep.simxGetObjectFloatParameter(self.sim_client, self.cam_handle, vrep.sim_visionfloatparam_far_clipping , vrep.simx_opmode_blocking);
            [res, resX]= vrep.simxGetObjectFloatParameter(self.sim_client, self.cam_handle, vrep.sim_visionintparam_resolution_x , vrep.simx_opmode_blocking);
            [res, resY]= vrep.simxGetObjectFloatParameter(self.sim_client, self.cam_handle, vrep.sim_visionintparam_resolution_y  , vrep.simx_opmode_blocking);
            [res, perspectiveAngle]= vrep.simxGetObjectFloatParameter(self.sim_client, self.cam_handle, vrep.sim_visionfloatparam_perspective_angle  , vrep.simx_opmode_blocking);

            self.nearClip = nearClip;
            self.farClip = farClip;
            
        end
                
        
    
        function [image, depth_img] = get_camera_data(self, vrep)

            % Get color image from simulation
            self.sim_client
            self.cam_handle
            [sim_ret, resolution, image] = vrep.simxGetVisionSensorImage2(self.sim_client, self.cam_handle, 0, vrep.simx_opmode_blocking);
            
            % Get depth image from simulation
            [sim_ret, resolution, depth_img] = vrep.simxGetVisionSensorDepthBuffer2(self.sim_client, self.cam_handle, vrep.simx_opmode_blocking);
            zNear = 0.01;            
            zFar = 10;
            depth_img = depth_img * (zFar - zNear) + zNear;  
            
        end
        
      function [pos, thetaL, thetaS] = estimatePosOrientation(rob)

            % limits of conveyo - brick plane: xmin xmax, ymin ymax, zmain zmax
            rob.workspace_limits = [[-1.5, -0.5]; [0.26, 0.6]; [0.21, 0.3]];

            % get point cloud
            pc = rob.get_point_cloud;
            %figure
            %pcshow(pc.Location)
            %title('Filtered point cloud');
            %xlabel('x'); ylabel('y')

            % fit a plane
            referenceVector = [0,0,1];
            maxDistance = 0.001;
            maxAngularDistance = 0.5; % deg
            [model1, inlierIndices, outlierIndices] = pcfitplane(pc, maxDistance, referenceVector, maxAngularDistance);
            plane = select(pc, inlierIndices);
           % pcshow(plane.Location)
            %title('First Plane'); xlabel('x'); ylabel('y')

            % bounding box of plane
            p = plane.Location(:, 1:2);
            bb = minBoundingBox(p');
            zpos = mean(plane.Location(:,3));
            zpos = zpos - 0.01; % decrease a bit

            %plot(bb(1,:), bb(2,:))
            %pbaspect([1 1 1])
            %title('Bounding box'); xlabel('x'); ylabel('y')

            % vector of largest side in rectangle
            v = [ bb(:, 2) - bb(:, 1) bb(:, 3) - bb(:, 2) ];
            nv = [ norm(v(:,1))  norm(v(:,2)) ];
            [~, im]=max(nv);
            vmax = v(:, im);
            [~, im]=min(nv);
            vmin = v(:, im);
            po = mean(bb');
            
            %hold on
            %quiver(po(1), po(2), vo(1), vo(2))
            pos = [ po(1:2) zpos]; % x-y mean coordinate and mean z coordinate

            % estimate orientation by cross product: Largest side
            v1 = [1, 0, 0]; v2 = [vmax' 0]; 
            dv = dot(v2, v1)/norm(v2);
            if abs(dv-1) < 1e-5; dv = 1; end % accuracy problem in Matlab
            phi = acos( dv )*180/pi; % deg. system 
            cp = cross(v1, v2);
            thetaL = sign(cp(3))*phi; % orientation wrt to z axis

            % estimate orientation by cross product: Smallest side
            v1 = [1, 0, 0]; v2 = [vmin' 0]; 
            dv = dot(v2, v1)/norm(v2);
            if abs(dv-1) < 1e-5; dv = 1; end % accuracy problem in Matlab
            phi = acos( dv )*180/pi; % deg. system 
            cp = cross(v1, v2);
            thetaS = sign(cp(3))*phi; % orientation wrt to z axis
            
      end
      function pc = get_point_cloud(rob)
            
            % gets the point cloud within worspace and excluding the base plane

            % Get latest RGB-D image
            [color_img, depth_img] = rob.get_camera_data(rob.vrep);
            
            %imshow(color_img)
            %title('Original Picture')
            %figure
            %imshow(depth_img)
            %title('Depth Map')

            % 3D point cloud from depth images
            % [surface_pts, color_pts] = get_pointcloudv2(color_img, depth_img, rob.cam_intrinsics);
            surface_pts = get_pointcloud_nocolor(depth_img, rob.cam_intrinsics); 
            %figure
            %pcshow(surface_pts)
            %title('point cloud');
            [m, n] = size(surface_pts);

            % transform 3D point cloud from camera coordinates to robot coordinates
            spt = transpose ( transpose(euler2rotm(-rob.cam_orientation))*transpose(surface_pts)  + repmat(rob.cam_position', [1, m]) ); %
            
%             pca = pointCloud(spt);
%             ws = rob.workspace_limits';
%             roi = ws(:);
%             indices = findPointsInROI(pcd, roi);            
%             pc = select(pca, indices);


            % Filter out surface points outside heightmap boundaries
            ws = rob.workspace_limits;
            id_valid = and( and(and(and(and( spt(:,1) >= ws(1,1) , spt(:,1) < ws(1,2) ), spt(:,2) >= ws(2,1) ), spt(:,2) < ws(2,2) ), spt(:,3) < ws(3,2) ), spt(:,3) > ws(3,1)) ;
            spv = spt(id_valid, :);
            spv = double(spv);
%             spv = spt;

            % Filter out the base plane
%             a = 0.8;
%             zt = a*min( spv(:,3) ) + (1-a)*max( spv(:,3) );
%             id_valid = spt(:, 3) > zt;
%             spf = spt(id_valid, :);

            pc = pointCloud(spv);
            
      end
        
        
        function obj_positions = get_obj_positions(self, vrep)
            obj_positions = [];
            for i = 1:numel(self.object_handles)
                object_handle = self.object_handles(i);
                [sim_ret, object_position ] = vrep.simxGetObjectPosition(self.sim_client, object_handle, -1, vrep.simx_opmode_blocking);
                obj_positions  = [obj_positions; object_position];
            end
            
        end
        function rotat(self, heightmap_rotation_angle, vrep)
            [sim_ret, gripper_orientation] = vrep.simxGetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, vrep.simx_opmode_blocking);
            tool_rotation_angle=(mod(heightmap_rotation_angle, pi) ) - pi/2;
            if (tool_rotation_angle - gripper_orientation(2) > 0)
                rotation_step = 0.3;
            else
                rotation_step = -0.3;
            end
            %rotation_step = 0.3 if (tool_rotation_angle - gripper_orientation[1] > 0) else -0.3
            num_rotation_steps = floor( (tool_rotation_angle - gripper_orientation(2))/rotation_step )

            % Simultaneously move and rotate gripper
            for i =1:max(abs(num_rotation_steps))
                step_iter = i -1;
                ori = [pi/2, gripper_orientation(2) + rotation_step*min(step_iter,num_rotation_steps), pi/2];
                vrep.simxSetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, ori, vrep.simx_opmode_blocking);
            end
            vrep.simxSetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, [pi/2, tool_rotation_angle, pi/2], vrep.simx_opmode_blocking)
        end
        function move_to(self, tool_position, vrep)
             % sim_ret, UR5_target_handle = vrep.simxGetObjectHandle(self.sim_client,'UR5_target',vrep.simx_opmode_blocking)
            [sim_ret, UR5_target_position] = vrep.simxGetObjectPosition(self.sim_client, self.UR5_target_handle,-1, vrep.simx_opmode_blocking);

            move_direction = tool_position - UR5_target_position;            
            move_magnitude = norm(move_direction)
            move_step = 0.075*move_direction/move_magnitude;
            num_move_steps = floor(move_magnitude/0.075);

            for i = 1:num_move_steps
                pos = UR5_target_position + move_step;
                vrep.simxSetObjectPosition(self.sim_client, self.UR5_target_handle,-1, pos, vrep.simx_opmode_blocking);
                [sim_ret, UR5_target_position] = vrep.simxGetObjectPosition(self.sim_client,self.UR5_target_handle,-1,vrep.simx_opmode_blocking);
            end
            
            vrep.simxSetObjectPosition(self.sim_client, self.UR5_target_handle,-1, tool_position, vrep.simx_opmode_blocking);
 
        end
        

        function gripper_fully_closed =  close_gripper(self, vrep)
            gripper_motor_velocity = -0.5;
            gripper_motor_force = 100;
            [sim_ret, RG2_gripper_handle] = vrep.simxGetObjectHandle(self.sim_client, 'RG2_openCloseJoint', vrep.simx_opmode_blocking);
            [sim_ret, gripper_joint_position] = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle, vrep.simx_opmode_blocking);
            vrep.simxSetJointForce(self.sim_client, RG2_gripper_handle, gripper_motor_force, vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity(self.sim_client, RG2_gripper_handle, gripper_motor_velocity, vrep.simx_opmode_blocking);
            gripper_fully_closed = false;
            while (gripper_joint_position > 0.017) % Block until gripper is fully closed
                [sim_ret, new_gripper_joint_position] = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle, vrep.simx_opmode_blocking);
                % disp(gripper_joint_position)
                if new_gripper_joint_position >= gripper_joint_position
                    return 
                end
                gripper_joint_position = new_gripper_joint_position;
            
            end
            gripper_fully_closed = true;

            
        end
        
        function open_gripper(self, vrep)
            gripper_motor_velocity = 0.5;
            gripper_motor_force = 20;
            [sim_ret, RG2_gripper_handle] = vrep.simxGetObjectHandle(self.sim_client, 'RG2_openCloseJoint', vrep.simx_opmode_blocking);
            [sim_ret, gripper_joint_position] = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle, vrep.simx_opmode_blocking);
            vrep.simxSetJointForce(self.sim_client, RG2_gripper_handle, gripper_motor_force, vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity(self.sim_client, RG2_gripper_handle, gripper_motor_velocity, vrep.simx_opmode_blocking);
            while (gripper_joint_position < 0.025) % Block until gripper is fully open
                [sim_ret, gripper_joint_position] = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle, vrep.simx_opmode_blocking)
            end
            
        end
        
        function check_sim(self)

        % Check if simulation is stable by checking if gripper is within workspace
        [sim_ret, gripper_position] = self.vrep.simxGetObjectPosition(self.sim_client, self.RG2_tip_handle, -1, self.vrep.simx_opmode_blocking);
        sim_ok = (gripper_position(1) > self.workspace_limits(1,1) - 0.1) & ( gripper_position(1) < self.workspace_limits(1,2) + 0.1) & ( gripper_position(2) > self.workspace_limits(2,1) - 0.1 ) & ( gripper_position(2) < self.workspace_limits(2,2) + 0.1) & ( gripper_position(3) > self.workspace_limits(3,1) ) & ( gripper_position(3) < self.workspace_limits(3,2)  )
        if ~ sim_ok
            disp('Simulation unstable. Restarting environment.')
            self.restart_sim(self.vrep)
            self.add_objects()
        end
        
        end
        
        function outputArg = method1(obj,inputArg)
            outputArg = obj.Property1 + inputArg;
        end
        
    end % methods
    
end