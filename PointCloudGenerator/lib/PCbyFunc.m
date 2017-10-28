function ptCloud = PCbyFunc(f, x_space, y_space)
% Generate a point cloud from a f(x,y) function
% Parameters:
%   f: the generator function
%   x_space: x-axis generation space (used in meshgrid)
%   y_space: y-axis generation space (used in meshgrid)

%% Point Cloud Generation
[X,Y]=meshgrid(x_space,y_space);
Z=f(X,Y);
P=([X(:), Y(:), Z(:)].')';
P(P(:,3)==inf,:) = [];
ptCloud = pointCloud(P,'color', repmat([1 0 0],size(P,1),1));
end