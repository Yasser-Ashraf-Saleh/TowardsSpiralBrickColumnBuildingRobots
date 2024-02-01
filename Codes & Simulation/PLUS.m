matx=[[-2 0];[2 0];[0 0];[0 2];[0 0];[0 -2]];
angle_of_rotation=-4;
no_of_blocks=[5 2 2];
intersect_angle=[pi pi/2 pi/2];
total_no_blocks=sum(no_of_blocks);
shifts=open_shapes(matx,angle_of_rotation,no_of_blocks,intersect_angle);