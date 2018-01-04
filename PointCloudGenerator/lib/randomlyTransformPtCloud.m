function [ptCloud_P,T] = randomlyTransformPtCloud(ptCloud_Q,sigma,beta)
%% Apply Random Transformation
% Random transformation parameters
max_rot = [2*pi 2*pi 2*pi];
min_t = repmat(getDiameter(ptCloud_Q),3,1); % ensure no overlap
max_t = min_t*2;

[ptCloud_P, T] = ApplyRandomTransformation(ptCloud_Q, max_rot, min_t, max_t);

%% Add Noise
ptCloud_P = AddNoise(ptCloud_P, sigma);

%% Add Outliers
ptCloud_P = AddOutliers(ptCloud_P, beta);

%% Colors
ptCloud_Q.Color = uint8( repmat([255 0 0], ptCloud_Q.Count ,1) ); % red
ptCloud_P.Color = uint8( repmat([0 0 255], ptCloud_P.Count ,1) ); % green