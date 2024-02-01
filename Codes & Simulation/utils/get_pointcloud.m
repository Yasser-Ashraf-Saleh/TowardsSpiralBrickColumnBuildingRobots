function [cam_pts, rgb_pts] = get_pointcloud(color_img, depth_img, camera_intrinsics)

    % Get depth image size
    [im_h, im_w ] = size(depth_img);

    % Project depth into 3D point cloud in camera coordinates
    [pix_x, pix_y] = meshgrid( linspace(0, im_w-1, im_w), linspace(0, im_h - 1, im_h ) );
    cam_pts_x = ( pix_x - camera_intrinsics(1,3) ).*(depth_img/camera_intrinsics(1,1) );
    cam_pts_y = ( pix_y - camera_intrinsics(2,3) ).*( depth_img/camera_intrinsics(2,2) );
    cam_pts_z = depth_img;
    
    cam_pts_x = reshape(cam_pts_x, im_h*im_w, 1);
    cam_pts_y = reshape(cam_pts_y, im_h*im_w, 1);
    cam_pts_z = reshape(cam_pts_z, im_h*im_w, 1);

    % Reshape image into colors for 3D point cloud
    rgb_pts_r = color_img(:, :, 1);
    rgb_pts_g = color_img(:, :, 2);
    rgb_pts_b = color_img(:, :, 3);
    
    rgb_pts_r = reshape(rgb_pts_r, im_h*im_w,1);
    rgb_pts_g = reshape(rgb_pts_g, im_h*im_w,1);
    rgb_pts_b = reshape(rgb_pts_b, im_h*im_w,1);

    cam_pts =  [cam_pts_x, cam_pts_y, cam_pts_z]; 
    rgb_pts = [rgb_pts_r, rgb_pts_g, rgb_pts_b];
%     cam_pts = np.concatenate( (cam_pts_x, cam_pts_y, cam_pts_z), axis=1);
%     rgb_pts = np.concatenate( (rgb_pts_r, rgb_pts_g, rgb_pts_b), axis=1);

end    
 