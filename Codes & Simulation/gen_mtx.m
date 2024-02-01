function shifts=gen_mtx(vertices,no_side_blocks,angle_of_rotation)
lenghts_slope=[];
block_length=0.1;
block_width=0.05;
slope_list=[]
    for i=1:length(vertices)
        if i~=length(vertices)
            vec=vertices(i,:)-vertices(i+1,:);
        else
            vec=vertices(i,:)-vertices(1,:);
        end
        slope=mod(atan(vec(2)/vec(1))+pi/2,pi);
        slope_list(end+1)=atan2(vec(2),vec(1));
        len=norm(vec);
        lenghts_slope(i,:)=[len,slope];
    end
    vertices=((1)/lenghts_slope(1,1))*vertices; %if all sides are equel
    verc_before_margin=vertices;
    vertices=[vertices,lenghts_slope(:,2)]

    %getting the angle between sides
    angle_list=[];
    Z=slope_list;
    for i=1:length(Z)
        if i<length(Z)
           angle_list(end+1)=180*(Z(i+1)-Z(i))/pi+180;
        else
           angle_list(end+1)=180*(-Z(i)+Z(1))/pi+180;
        end
        %angle_list(end)=mod(angle_list(end),2*pi);
        %angle_list(end)=mod(angle_list(end),pi);
        angle_list(end)=mod(angle_list(end),360);
        if angle_list(end)<0
             angle_list(end)=angle_list(end)+360;
        end
        if angle_list(end)>=180
             angle_list(end)=-angle_list(end)+360;
        end
        angle_list(end)=angle_list(end)*pi/180

    end


    %adding_margins
    margin_list=[];
    for i=1:length(vertices(:,1))
         m=1/tan(angle_list(i)/2);
         %margin=m*block_width+no_side_blocks*block_length+(no_side_blocks-1)*0.01+0.02;%square
         margin=m*block_width+no_side_blocks*block_length+(no_side_blocks-1)*0.01+0.01;
         vertices(i,1:2)=vertices(i,1:2)*margin;
         margin_list(i)=margin;
    end
    %plot(polyshape(vertices(:,1),vertices(:,2)))
    %hold on
    %%getting the origin of the blocks
    shifts=[];
    Z=vertices(:,3);
    for i=1:length(vertices(:,1))
       if i~=length(vertices)
            vec=vertices(i+1,1:2)-vertices(i,1:2);
            vec2=verc_before_margin(i+1,1:2)-verc_before_margin(i,1:2);
        else
            vec=vertices(1,1:2)-vertices(i,1:2);
            vec2=verc_before_margin(1,1:2)-verc_before_margin(i,1:2);
       end
        for j=1:no_side_blocks
            if j==1
                    shifts(end+1,1:2)=vec*(block_width)+vertices(i,1:2)+((2*j-1)/(2*no_side_blocks))*vec;
                    shifts(end,3)=Z(i);
            else
                    shifts(end+1,1:2)=vertices(i,1:2)+(j*block_length)*vec2;
                    shifts(end,3)=Z(i);
            end

        end
    end
    %plot(polyshape(shifts(:,1),shifts(:,2)));
    %hold on
    %getting the centre
    Centre=[sum(vertices(:,1))/length(vertices(:,1)),sum(vertices(:,2))/length(vertices(:,2))]
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