function [best_pix_ind, mbest ]= grasp_heuristic(depth_heightmap)

        num_rotations = 16;
        [R, C]= size(depth_heightmap);

        for cr = 1:num_rotations
            rotate_idx  = cr -1;
            rotated_heightmap = imrotate( depth_heightmap, rotate_idx*(360/num_rotations) , 'crop');
%             size(rotated_heightmap)            
            valid_areas = zeros(size(rotated_heightmap));
            shift1 = circshift(rotated_heightmap, [0, -25]); shift1(:,(end-24):end) = 0;
            shift2 = circshift(rotated_heightmap, [0, 25]); shift2(:,1:25) = 0;
            valid_areas( and(rotated_heightmap - shift1 > 0.02, rotated_heightmap - shift2 > 0.02) ) = 1;
            % valid_areas = np.multiply(valid_areas, rotated_heightmap)
            blur_kernel = ones(25)/9;
            valid_areas = conv2(valid_areas, blur_kernel, 'same');
%             size(valid_areas)
            tmp_grasp_predictions = imrotate(valid_areas, -rotate_idx*(360.0/num_rotations), 'crop');
%             size(tmp_grasp_predictions)
            [m, ind] = max(tmp_grasp_predictions(:));
            
            if  rotate_idx == 0
                ci = floor((ind-1)/R) + 1;
                ri = ind - R*(ci-1); 
                mbest = m;
            else
                if m > mbest 
                    mbest = m;
                    ci = floor( (ind-1)/R ) + 1;
                    ri = ind - R*(ci-1);
                end
            end
        end
       best_pix_ind = [ci, ri];
       
end

        
