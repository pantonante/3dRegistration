function ptCloud_noisy = AddNoise( ptCloud, sigma )

N = ptCloud.Count;
diameters = [
    abs(ptCloud.XLimits(1)-ptCloud.XLimits(2)) ...
    abs(ptCloud.YLimits(1)-ptCloud.YLimits(2))...
    abs(ptCloud.ZLimits(1)-ptCloud.ZLimits(2))];
abs_sigma = sigma*mean(diameters);
noise = normrnd(0, abs_sigma, [N,3]);
pt_noisy = ptCloud.Location + noise;
ptCloud_noisy = pointCloud(pt_noisy);

end

