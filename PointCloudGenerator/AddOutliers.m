function ptCloud_out = AddOutliers( ptCloud, ratio)

idx=randperm(ptCloud.Count);
idx=idx(1:floor(ptCloud.Count*ratio));
n_x = (randi([0,1], length(idx), 1)*2 - 1) .* ...
      (ptCloud.XLimits(1) + ptCloud.XLimits(2)*randn(length(idx),1));
n_y = (randi([0,1], length(idx), 1)*2 - 1) .* ...
      (ptCloud.YLimits(1) + ptCloud.YLimits(2)*randn(length(idx),1));
n_z = (randi([0,1], length(idx), 1)*2 - 1) .* ...
      (ptCloud.ZLimits(1) + ptCloud.ZLimits(2)*randn(length(idx),1));

N = [n_x, n_y, n_z];  
P=ptCloud.Location;
P(idx,:)=P(idx,:)+N;
ptCloud_out=pointCloud(P);

end

