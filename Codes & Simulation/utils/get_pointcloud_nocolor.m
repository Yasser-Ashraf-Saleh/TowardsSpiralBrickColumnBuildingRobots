function  cloud = get_pointcloud_nocolor(depth, camera_intrinsics)

K = camera_intrinsics;

%depth is depth image in double format
Sd = size(depth);
[X, Y] = meshgrid(1:Sd(2), 1:Sd(1));

%K is calibration matrix
X = X - K(1,3) + 0.5;
Y = Y - K(2,3) + 0.5;
% X = X - K(1,3) ;
% Y = Y - K(2,3) ;
XDf = depth/K(1,1);
YDf = depth/K(2,2);
X = -X .* XDf; % negative sign is due to pinhole cameral model
Y = -Y .* YDf; % negative sign is due to pinhole cameral model
XY = cat(3, X, Y);
cloud = cat(3, XY, depth);
% cloud = reshape(cloud,[],3) / 1000;
cloud = reshape(cloud, [], 3);


end
