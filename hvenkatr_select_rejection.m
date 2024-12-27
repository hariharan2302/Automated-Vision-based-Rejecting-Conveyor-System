clc;
close all;
clear all;

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
[res,sensor_B]=sim.simxGetObjectHandle(clientID,'Proximity_sensor',sim.simx_opmode_blocking);
[res,sensor_A]=sim.simxGetObjectHandle(clientID,'Proximity_sensor0',sim.simx_opmode_blocking);
[res,my_actuator]=sim.simxGetObjectHandle(clientID,'Prismatic_joint',sim.simx_opmode_blocking);
disp(['Linear actuator handle: ', num2str(my_actuator)]);



% The -1 here refers that we are concerned about the world coordinates
[res,pos]=sim.simxGetObjectPosition(clientID,obj1,-1,sim.simx_opmode_streaming);
[res,pos1]=sim.simxGetObjectPosition(clientID,obj2,-1,sim.simx_opmode_buffer);
[res,pos3]=sim.simxGetObjectPosition(clientID,obj3,-1,sim.simx_opmode_buffer);
[res,pos4]=sim.simxGetObjectPosition(clientID,obj4,-1,sim.simx_opmode_buffer);
[res,pos5]=sim.simxGetObjectPosition(clientID,obj5,-1,sim.simx_opmode_buffer);
[res,pos6]=sim.simxGetObjectPosition(clientID,obj6,-1,sim.simx_opmode_buffer);
[res,pos7]=sim.simxGetObjectPosition(clientID,obj7,-1,sim.simx_opmode_buffer);
[res,pos8]=sim.simxGetObjectPosition(clientID,obj8,-1,sim.simx_opmode_buffer);
[res, position] = sim.simxGetJointPosition(clientID, my_actuator, sim.simx_opmode_blocking);
disp(['Current joint position: ', num2str(position)]);

dummy =0;
log_data =[];
check_std_array = [];
while true
    [returncode_1,detec_state_sensor_B,detec_pt_sensor_B,detect_objhandle_sensor_B,detec_SurfNorVec_sensor_B]=sim.simxReadProximitySensor(clientID,sensor_B,sim.simx_opmode_streaming);
    [returncode_2,detec_state_sensor_A,detec_pt_sensor_A,detect_objhandle_sensor_A,detec_SurfNorVec_sensor_A]=sim.simxReadProximitySensor(clientID,sensor_A,sim.simx_opmode_streaming);
    if detec_state_sensor_B ==1 && dummy ==0
        dummy =1;
        disp('Object detected on SENSOR B')
        log_data = [log_data,detec_pt_sensor_B];
    end
    if detec_state_sensor_B ==0 && dummy==1
        check_var = std(log_data);
        disp(['std:',num2str(check_var)])
        check_std_array = [check_std_array, check_var];
        log_data =[];
        dummy=0;
    end
        % disp(['detected obj handle:',detect_objhandle_sensor_B])
        % disp(['detectedpoint:',detec_pt_sensor_B])
        % disp(['standard deviation of obj',num2str(check_var
    if detec_state_sensor_A ==1
        if check_var< 0.09
            disp('Object detected on SENSOR A')
            [returncode_2]=sim.simxSetJointPosition(clientID,my_actuator,0.2,sim.simx_opmode_oneshot); % here i am extending the actuator 
            % disp('Actuator Extended')
            pause(0.01)
            [returncode_3]=sim.simxSetJointPosition(clientID,my_actuator,0,sim.simx_opmode_oneshot); % here i am retracting the actuator 
            % disp('Actuator Retracted')
            pause(0.01)
        end
    end
end    
[returnCode]=sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)