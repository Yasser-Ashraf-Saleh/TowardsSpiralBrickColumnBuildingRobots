vertices=[[0 0];[cos(60*pi/180) sin(60*pi/180)];[1 0]];
%plot(polyshape(vertices(:,1),vertices(:,2)));
%hold on
no_of_block_sides=2;
angle_of_rotation=4;
shifts=gen_mtx(vertices,no_of_block_sides,angle_of_rotation);
total_no_blocks=3*no_of_block_sides;