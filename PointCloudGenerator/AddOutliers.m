function ptCloud_out = AddOutliers( ptCloud, ratio , cube)
% Add outliers to the point cloud
% Parameters:
%   ptCloud - Point Cloud
%   ratio - the percentage of outliers
%   cube (optional) - cube sizes in which the outliers lies, centered 
%     in pt. cloud. If omitted is the diameter of the pt. cloud.

if(nargin<3)
    cube = [ptCloud.XLimits(2) - ptCloud.XLimits(1),...
        ptCloud.YLimits(2) - ptCloud.YLimits(1),...
        ptCloud.ZLimits(2) - ptCloud.ZLimits(1)];
end

centers = [(ptCloud.XLimits(2) + ptCloud.XLimits(1))/2,...
        (ptCloud.YLimits(2) + ptCloud.YLimits(1))/2,...
        (ptCloud.ZLimits(2) + ptCloud.ZLimits(1))/2];

idx=randperm(ptCloud.Count);
idx=idx(1:floor(ptCloud.Count*ratio));

o_x = centers(1)-cube(1)/2 + rand(length(idx), 1)*cube(1);
o_y = centers(2)-cube(2)/2 + rand(length(idx), 1)*cube(2);
o_z = centers(3)-cube(3)/2 + rand(length(idx), 1)*cube(3);
O = [o_x, o_y, o_z];  

P=ptCloud.Location;
P(idx,:)=O;
ptCloud_out=pointCloud(P);

end

