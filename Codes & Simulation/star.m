t=37+(sqrt(3)*5);%prameters for the star shape
nn=t/(0.6641);%prameters for the star shape
%shifts is the vertices cordinates
vertices=0.2*nn*[[4.755282581 1.545084972];[1.469463131 2.022542486]; [0 5];[-1.469463131 2.022542486]; [-4.755282581 1.545084972]; [-2.377641291 -0.7725424859];[-2.938926261 -4.045084972];[0 -2.5] ;[2.938926261 -4.045084972]; [2.377641291 -0.7725424859]];
no_of_block_sides=1;
angle_of_rotation=4;
shifts=gen_mtx(vertices,no_of_block_sides,angle_of_rotation);
total_no_blocks=10*no_of_block_sides;