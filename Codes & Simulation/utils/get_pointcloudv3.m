function P = get_pointcloudv3(color_img, depth_img, Pinv)

[m, n] = size(depth_img);

P = [];
for i = 1:m
    for j = 1:n
        p = Pinv*[i ; j; depth_img(i,j)];
        P = [P, p];
    end
end



end