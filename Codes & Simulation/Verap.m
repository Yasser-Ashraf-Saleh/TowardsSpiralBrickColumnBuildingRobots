clear 
close all
clc

vrep=remApi('remoteApi');
%close_every_thing
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5)
% start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
[returnCode, Left_Motor] = vrep.simxGetObjectHandle(clientID,'nakedCar_motorLeft',vrep.simx_opmode_blocking);
[returnCode, Right_Motor] = vrep.simxGetObjectHandle(clientID,'nakedCar_motorRight',vrep.simx_opmode_blocking);
[returnCode, RS] = vrep.simxGetObjectHandle(clientID,'Right_Light_sensor',vrep.simx_opmode_blocking);
[returnCode, MS] = vrep.simxGetObjectHandle(clientID,'middle_light_sensor',vrep.simx_opmode_blocking);
[returnCode, LS] = vrep.simxGetObjectHandle(clientID,'LLS',vrep.simx_opmode_blocking);
[returnCode, RStearing] = vrep.simxGetObjectHandle(clientID,'nakedCar_steeringRight',vrep.simx_opmode_blocking);
[returnCode, LStearing] = vrep.simxGetObjectHandle(clientID,'nakedCar_steeringLeft',vrep.simx_opmode_blocking);

while 1
    [returnCode1,dataR,resultR]=vrep.simxReadVisionSensor(clientID,RS,vrep.simx_opmode_blocking);
    [returnCode1,dataL,resultL]=vrep.simxReadVisionSensor(clientID,LS,vrep.simx_opmode_blocking);
    [returnCode1,dataM,resultM]=vrep.simxReadVisionSensor(clientID,MS,vrep.simx_opmode_blocking);
    [res,rs,img]=vrep.simxGetVisionSensorImage2(clientID,RS,0,vrep.simx_opmode_oneshot_wait);
    size(img)
    if resultR(11)<0.5 
        wl = 3;
        wR=0;
        AL=0
        AR=1
    elseif resultL(11)<0.5  
        wl = 0;
        wR=3;
        AL=1
        AR=0
    elseif resultM(11)<0.5
        wl = 5;
        wR=5;
        AL=0
        AR=0
    else
        wl=wl
        wR=wR
        AR=0
        AL=0
    end
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, Left_Motor, wl, vrep.simx_opmode_oneshot);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, Right_Motor, wR, vrep.simx_opmode_oneshot);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID, RStearing, AR, vrep.simx_opmode_oneshot);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID, LStearing, AL, vrep.simx_opmode_oneshot);

    [wl,wR]
end
