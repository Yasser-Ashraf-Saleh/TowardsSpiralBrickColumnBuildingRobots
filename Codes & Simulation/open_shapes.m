function shifts=open_shapes(matx,angle_of_rotation,no_of_blocks,intersect_angle)
shifts=[];
block_length=0.1;
block_width=0.05;
for i=1:length(no_of_blocks)
    slope=(matx(2*i-1,2)-matx(2*i,2))/(matx(2*i-1,1)-matx(2*i,1));
    th=mod(atan(slope)+pi/2,pi);
    margin=1/tan(intersect_angle(i)/2);
    L=norm(matx(2*i-1,:)-matx(2*i,:));
    if margin~=0
         S=no_of_blocks(i)*block_length+0.005*(no_of_blocks(i)-1)+margin*block_width;
    else
         S=no_of_blocks(i)*block_length+0.005*(no_of_blocks(i)-1);
    end
    u=[matx(2*i-1,:);matx(2*i,:)]*(1/L);
    m=u*S
    vec=u(2,:)-u(1,:)
    for j=1:no_of_blocks(i)
        if margin~=0 | i~=1 | j==1
            shifts(end+1,1:2)=m(1,:)+vec*(block_length*(j-0.5)+0.005*(j))+(block_width/2)*vec;
        else
            shifts(end+1,1:2)=m(1,:)+vec*(block_length*(j-0.5)+0.005*(j-1));
        end
        shifts(end,3)=th;
    end
end

Centre=[sum(shifts(:,1))/length(shifts(:,1)),sum(shifts(:,2))/length(shifts(:,2))];
th=angle_of_rotation*pi/180;
X=shifts(:,1);
Y=shifts(:,2);
Z=shifts(:,3);
for i=1:30
   for j=1:length(X)
       a=X(j);
       b=Y(j);
       X(j)=(-Centre(1)+a)*cos(th)-(-Centre(2)+b)*sin(th);
       Y(j)=(-Centre(1)+a)*sin(th)+(-Centre(2)+b)*cos(th);
       Z(j)=mod(Z(j)+th,pi);
       X(j)=X(j)+Centre(1);
       Y(j)=Y(j)+Centre(2);
       shifts(end+1,:)=[X(j),Y(j),Z(j)];
   end
        %pgon=polyshape(X,Y);
        %plot(pgon);
        %hold on
end
   shifts(:,1)=shifts(:,1)-Centre(1);
   shifts(:,2)=shifts(:,2)-Centre(2);
   shifts
end