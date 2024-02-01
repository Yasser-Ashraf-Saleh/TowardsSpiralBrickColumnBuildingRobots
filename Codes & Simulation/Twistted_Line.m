matx=[[-5 0];[5 0];[-5 1.9];[5 1.9]];
angle_of_rotation=-4;
no_of_blocks=[4 4];
intersect_angle=[pi pi];
total_no_blocks=sum(no_of_blocks);
shifts=open_shapes(matx,angle_of_rotation,no_of_blocks,intersect_angle);