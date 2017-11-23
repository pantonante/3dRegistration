function pc_info(ptCloud, radius)
%PC_INFO Return some point cloud information
% Inputs: 
%   - ptCloud: Filename or Matlab point cloud)
%   - radius: (optional) the nearest neighbour distance radius
% Prints:
%   - Number of points
%   - Diameter
%   - Min and average nearest neighbour distance
%   - If radius is defined, avg. number of points in the specified radius


if ischar(ptCloud)
    ptCloud = pcread(ptCloud);
end

N = ptCloud.Count;
D = zeros(N,1);
if nargin>1
    pts = zeros(N,1);
end

disp(['Number of points: ', num2str(ptCloud.Count)])
disp(['Diameter: ', num2str(getDiameter(ptCloud))])

for i=1:N
    p =  ptCloud.Location(i,:);    
    d = ptCloud.Location(1:end ~= i,:)-p;
    dist = sqrt(sum(d.^2,2));
    if nargin>1
        ids = find(dist<=radius);
        pts(i)=length(ids);
    end
    D(i) = min(dist);
end

disp(['Min. nearest neighbour distance: ', num2str(min(D))])
disp(['Avg. nearest neighbour distance: ', num2str(mean(D))])

if nargin>1
    disp(['Avg. number of points in ', num2str(radius),...
        ' radius: ', num2str(mean(pts))])
end

end