plyFile = 'bunny/reconstruction/bun_zipper_res3.ply';
sigma = 0.015;

%% Read PLY
ptCloud_Q=pcread(plyFile);
ptCloud_Q.Color = uint8( repmat([255 0 0], ptCloud_Q.Count ,1) );
disp(['Number of points: ', num2str(ptCloud_Q.Count)])

%% Apply Random Transformation
% Random transformation parameters
max_rot = [pi/2 pi/2 pi/2];
min_t = [
    abs(ptCloud_Q.XLimits(1)-ptCloud_Q.XLimits(2)) ...
    abs(ptCloud_Q.YLimits(1)-ptCloud_Q.YLimits(2))...
    abs(ptCloud_Q.ZLimits(1)-ptCloud_Q.ZLimits(2))];
max_t = min_t*1.3;

ptCloud_P = ApplyRandomTransformation(ptCloud_Q, max_rot, min_t, max_t);
ptCloud_P.Color = uint8( repmat([0 0 255], ptCloud_P.Count ,1) );

%% Add Noise
ptCloud_P = AddNoise(ptCloud_P, sigma);

%% Add Outliers
ptCloud_P = AddOutliers(ptCloud_P, 0.007);

%% SHOW
pcshow(ptCloud_Q)
hold on
pcshow(ptCloud_P)