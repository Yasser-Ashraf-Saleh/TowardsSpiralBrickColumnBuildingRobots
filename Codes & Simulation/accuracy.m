Actual_pos=[];
Actual_or=[];
for i=1:length(rob.object_handles)
    [ret,Actual_pos(end+1,:)]= rob.vrep.simxGetObjectPosition(rob.sim_client,rob.object_handles(i), -1, rob.vrep.simx_opmode_blocking);
    [ret,Actual_or(end+1,:)]= rob.vrep.simxGetObjectOrientation(rob.sim_client,rob.object_handles(i), -1, rob.vrep.simx_opmode_blocking);
    
end
shifts(:,1)=shifts(:,1)-0.25;
shifts(:,2)=shifts(:,2)-0.08;
shifts(:,3)=shifts(:,3)-pi/2;
Pos_Error=shifts(1:length(Actual_pos),1:2)-Actual_pos(:,1:2);
Ori_Error=tan(shifts(1:length(Actual_or),3)-Actual_or(:,3));
