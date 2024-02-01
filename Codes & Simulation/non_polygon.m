

%convention --- 
%an input array [function(x), xstart, xend]
syms x
polynomials=[[x^2-(0.09) -0.3 0.3];[-x^2+(0.09) -0.3 0.3]];
x_mat=[];
y_mat=[];
p_polynomials= polynomials; 
v=1;
for z= 1: size(polynomials,1)
    x= polynomials(z, 2:3);
    val = subs(polynomials(z,1));
    p_polynomials(z,4:5)= val ;
end
o_polynomials=[p_polynomials(1,:)];
for o = 1:size(p_polynomials,1)-1
    ref= o_polynomials(end, :);
    for v=2: size(p_polynomials,1)
        if p_polynomials(v,2)== ref(3)
            if p_polynomials(v,4)==ref(5)
                o_polynomials = [o_polynomials; p_polynomials(v,:)];
                break
            end
        end
        if p_polynomials(v,3)== ref(3)
            if p_polynomials(v,5)==ref(5)
                o_polynomials = [o_polynomials;[p_polynomials(v,1),p_polynomials(v,3),p_polynomials(v,2),p_polynomials(v,5),p_polynomials(v,4)]];
                break
            end
        end
    end
end

for i=1: size(polynomials,1)
    if o_polynomials(i,2)<o_polynomials(i,3)
        x= o_polynomials(i,2):0.01:o_polynomials(i,3);
    else
        x= o_polynomials(i,2):-0.01:o_polynomials(i,3);
        end
    x_mat = [x_mat, x];
    y= subs(polynomials(i,1));
    y_mat= [y_mat, y ];
    plot(x_mat(1,:),y_mat(1,:));
    hold on 
end

%%%search 
st_array_x=[];
st_array_y=[];
end_array_x=[];
end_array_y=[];
origins_array=[];
origins_theta=[];
ks=[]
lengthh=[];
cs=0;
aux_st=[];
w= 0.05;
for j = 1:size(x_mat,1)
    s= ['great',num2str(j)]
    k=0;
    e=0;
    s=1;
    cs=0;
    st_array_y=[st_array_y; y_mat(j,1)];
    st_array_x=[st_array_x; x_mat(j,1)];
    end_array_x=[end_array_x;[0]];
    end_array_y=[end_array_y;[0]];
    while true
        if k== length(x_mat(j,:))
            e1 = 'exceeded_1'
            lengthh=[lengthh, cs]; 
            break
        end
        k = k+1;
        st_point= [st_array_x(end,end),st_array_y(end,end)];
        end_point= [x_mat(j,k), y_mat(j,k)];
        vector = end_point - st_point;
        brick = norm(vector);
        if brick-0.1>=0
            s1='success_1'
            ks=[ks , k]; 
            e=e+1
            cs=cs+1;
            end_array_x(end,e)= end_point(1);
            end_array_y(end,e)= end_point(2);
            x_ori = (((end_point(1)-st_point(1))/2)+st_point(1));
            y_ori = (((end_point(2)-st_point(2))/2)+st_point(2));
            sl= end_point- st_point;
            theta_ori= atan2(sl(2), sl(1)) ;
            origins_array= [origins_array; [x_ori ,y_ori]];
            origins_theta= [origins_theta; theta_ori];
            comp_vec_st= end_point;
            while true
                if k== length(x_mat(j,:))  
                    e2 = 'exceeded_2'
                    lengthh=[lengthh, cs];
                    break
                end
                k=k+1; 
                comp_vec_end= [x_mat(j,k), y_mat(j,k)];
                comp_vec = comp_vec_end- comp_vec_st;
                vec_length = norm(comp_vec);               
                if vec_length-0.1>=0
                    s2= 'success_2'     
                    com_vecs = dot(vector, comp_vec);
                    th = 180 - (acos(com_vecs/(brick*vec_length))*180/pi);
                    margin = (sin((90 -(0.5*th))*pi/180)*w)+0.012; 
                    aux_st= [aux_st; k];
                    k= ks(end);
                    mar_st= end_point;
                    while true 
                        if k== length(x_mat(j,:))
                            e3 = 'exceeded_3'
                            lengthh=[lengthh, cs];
                            break
                        end    
                        k= k+1 
                        mar_end = [x_mat(j,k), y_mat(j,k)];
                        mar_vec = mar_end- mar_st; 
                        mar_len= norm(mar_vec);
                        if mar_len-margin>=0 
                            s5= 'success_3'
                            s=s+1;
                            st_array_x(end,s)= mar_end(1); 
                            st_array_y(end,s)= mar_end(2);
                      
                            k= aux_st(end);
                            
                            break 
                         
                        end
                    end
                    break 
                 
                end      
            end
                 
        end
    end        
end

X= []
Y= []
for i= 1: size(st_array_x, 2)
    X= [X; [st_array_x(i)];[end_array_x(i)]];
    Y= [Y; [st_array_y(i)];[end_array_y(i)]];
end 
xx=X(:,1);
yy=Y(:,1);
xd=double(xx); 
yd = double(yy);
xx_ori= double(origins_array(:,1));
yy_ori= double(origins_array(:,2));
pol2= polyshape(xx_ori,yy_ori);
pol= polyshape(xd,yd);
plot(pol);
hold on 
plot(pol2)

Centre = [sum(xx_ori)/size(xx_ori,1),sum(yy_ori)/size(yy_ori,1)];%this kis the centre of the origins (it's similar to that of the original shape)
zz = origins_theta;
origins_array =[origins_array,zz];
%%%%%%%%%%%%%%%%%%%%%%%%


th=-4*pi/180;
for i=1:30
    for j=1:size(xx_ori,1)
        a=xx_ori(j);
        b=yy_ori(j);
        xx_ori(j)=(-Centre(1)+a)*cos(th)-(-Centre(2)+b)*sin(th);
        yy_ori(j)=(-Centre(1)+a)*sin(th)+(-Centre(2)+b)*cos(th);
        zz(j)=zz(j)+th;
        xx_ori(j)=xx_ori(j)+Centre(1);
        yy_ori(j)=yy_ori(j)+Centre(2);
        origins_array(end+1,:)=[xx_ori(j),yy_ori(j),zz(j)];
    end
    pgon=polyshape(xx_ori,yy_ori);
    plot(pgon);
    hold on
end
shifts=[];
[r w]=size(origins_array);
for i=1:r
    for j=1:w
        if j==3
            shifts(i,j)=mod(origins_array(i,j)+pi/2,pi);
        else
            shifts(i,j)=origins_array(i,j);
        end
    end
end
total_no_blocks=9;




