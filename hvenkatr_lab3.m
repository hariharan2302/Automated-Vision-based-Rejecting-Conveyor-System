clc;
clear all;
close all;

disp("program started (debugging purpose)");
sim=remApi('remoteApi'); % here using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case to close all opened connections which could be opened previously
clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

% i am checking whether the API connection is successful
if clientID >-1
    disp('Connected to remote API server');
end

% Starting my simulation
[returnCode]=sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot)
% Now fetching all the object handles in the scene
[res,obj1]=sim.simxGetObjectHandle(clientID,'Cuboid',sim.simx_opmode_blocking); 
[res,obj2]=sim.simxGetObjectHandle(clientID,'Cylinder',sim.simx_opmode_blocking);
[res,obj3]=sim.simxGetObjectHandle(clientID,'Cylinder0',sim.simx_opmode_blocking);
[res,obj4]=sim.simxGetObjectHandle(clientID,'Cuboid0',sim.simx_opmode_blocking);
[res,obj5]=sim.simxGetObjectHandle(clientID,'Cylinder1',sim.simx_opmode_blocking);
[res,obj6]=sim.simxGetObjectHandle(clientID,'Cuboid1',sim.simx_opmode_blocking);
[res,obj7]=sim.simxGetObjectHandle(clientID,'Cylinder3',sim.simx_opmode_blocking);
[res,obj8]=sim.simxGetObjectHandle(clientID,'Cylinder2',sim.simx_opmode_blocking);
%[res,sensor_B]=sim.simxGetObjectHandle(clientID,'Proximity_sensor',sim.simx_opmode_blocking);
%[res,sensor_A]=sim.simxGetObjectHandle(clientID,'Proximity_sensor0',sim.simx_opmode_blocking);
[res,my_actuator]=sim.simxGetObjectHandle(clientID,'Prismatic_joint',sim.simx_opmode_blocking);
[res,vision_handle]=sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_blocking);
%disp(['Linear actuator handle: ', num2str(my_actuator)]);
%disp(['vision sensor handle: ', num2str(vision_handle)]);



% The -1 here refers that we are concerned about the world coordinates
[res,pos]=sim.simxGetObjectPosition(clientID,obj1,-1,sim.simx_opmode_streaming);
[res,pos1]=sim.simxGetObjectPosition(clientID,obj2,-1,sim.simx_opmode_buffer);
[res,pos3]=sim.simxGetObjectPosition(clientID,obj3,-1,sim.simx_opmode_buffer);
[res,pos4]=sim.simxGetObjectPosition(clientID,obj4,-1,sim.simx_opmode_buffer);
[res,pos5]=sim.simxGetObjectPosition(clientID,obj5,-1,sim.simx_opmode_buffer);
[res,pos6]=sim.simxGetObjectPosition(clientID,obj6,-1,sim.simx_opmode_buffer);
[res,pos7]=sim.simxGetObjectPosition(clientID,obj7,-1,sim.simx_opmode_buffer);
[res,pos8]=sim.simxGetObjectPosition(clientID,obj8,-1,sim.simx_opmode_buffer);
[res, position1] = sim.simxGetJointPosition(clientID, my_actuator, sim.simx_opmode_blocking);
[res, position2] = sim.simxGetJointPosition(clientID, vision_handle, sim.simx_opmode_blocking);
%disp(['Current joint position: ', num2str(position1)]);
%disp(['Current vision sensor position: ', num2str(position2)]);

for i =  1:100000
    [returncode1,resolution,my_img_mat] = sim.simxGetVisionSensorImage2(clientID, vision_handle,0,sim.simx_opmode_streaming);
    if ~isempty(my_img_mat)
        red_channel = my_img_mat(:,:,1);
        green_channel = my_img_mat(:,:,2);
        blue_channel = my_img_mat(:,:,3);
        my_binary_img = red_channel >200 | green_channel>200 | blue_channel>200;
        % Defining my target points 
        my_img_mat(8,18,3) = 255; % 8,18
        my_img_mat(24,18,3) = 255; %24,18
        if my_binary_img(8,18) ==1 && my_binary_img(24,18)==1
        %disp('Object Detected')
        % Identifing the Object's COLOR (changed the simulation dt = 0.015
        % in coppelia sim)
            obj_color = '';
            if my_img_mat(8,18,1)>=200 && my_img_mat(24,18,1)>=200
                obj_color = 'RED';
            elseif my_img_mat(8,18,2)>=200 && my_img_mat(24,18,2)>=200
                obj_color = 'GREEN';
            elseif my_img_mat(8,18,3)>=200 && my_img_mat(24,18,3)>=200
                obj_color = 'BLUE';
            end
            %disp(['Color of my Object:',obj_color]);
            pause(0.2)
            % Identifing Object shape and Size of the object
            stats = regionprops(my_binary_img,'Area','BoundingBox','Circularity');
            if ~isempty(stats)
                area_val = stats.Area;
                bbox_val = stats.BoundingBox; % it will return [x,y,width,height] where x,y is the top left corner in the rectangular region
                circularity_val = stats.Circularity;
                width = bbox_val(3);
                height = bbox_val(4);
                ratio = width/height;
                if circularity_val > 0.9
                    obj_shape = 'Circle';
                elseif abs(ratio - 1) < 0.1
                    obj_shape = 'Square';
                else
                    obj_shape = 'Rectangle';
                end
                disp(['Shape of my Object:',obj_shape]);
                disp(['Size of my Object:',num2str(area_val)]);
                % Here is MY Rejection Logic (here i am rejecting the red
                % cylinders)
                reject_color = 'RED';
                reject_shape = 'Circle';
                if strcmp(reject_color,obj_color) && strcmp(reject_shape,obj_shape)&& area_val >=350
                                [returncode_2]=sim.simxSetJointPosition(clientID,my_actuator,0.2,sim.simx_opmode_oneshot); % here i am extending the actuator 
                                pause(0.1)
                                [returncode_3]=sim.simxSetJointPosition(clientID,my_actuator,0,sim.simx_opmode_oneshot); % here i am retracting the actuator 
                end
            end
        end
        subplot(2,2,1);
        imshow(my_img_mat)
        subplot(2,2,2);
        imshow(my_binary_img)
        %my_binary_img    
        pause(0.2)
    end
end
[returnCode]=sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);

