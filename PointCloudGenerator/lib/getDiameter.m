function [diameter, COG] = getDiameter( ptCloud )
%GETDIAMETER Return the point cloud diameter
%   Input: point cloud (filename or Matlab point cloud)
%   Outputs:
%      - diameter: diameter of the point cloud
%      - COG: center of gravity

if ischar(ptCloud)
    ptCloud = pcread(ptCloud);
end

COG = sum(ptCloud.Location)/ptCloud.Count;
P = ptCloud.Location-COG;
diameter = 2*max(sqrt(sum(P.^2,2))); % norm is not suitable for the purpose

end

