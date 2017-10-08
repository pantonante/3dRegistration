%% Point Cloud Generation
% Sombrero function
f = @(X,Y) (10*sin(sqrt(X.^2 + Y.^2)) + 0.1) ./ sqrt(X.^2 + Y.^2);
x_space = -8:.25:8;
y_space = -8:.25:8;
sigma = 0.05; % noise
beta = 3/100; % outliers percentage

%% Generation
ptCloud_Q = PCbyFunc(f,x_space,y_space);

%% Apply Random Transformation
% Random transformation parameters
max_rot = [2*pi 2*pi 2*pi];
min_t = [ % ensure no overlap
    abs(ptCloud_Q.XLimits(1)-ptCloud_Q.XLimits(2)) ...
    abs(ptCloud_Q.YLimits(1)-ptCloud_Q.YLimits(2))...
    abs(ptCloud_Q.ZLimits(1)-ptCloud_Q.ZLimits(2))];
max_t = min_t*2;

ptCloud_P = ApplyRandomTransformation(ptCloud_Q, max_rot, min_t, max_t);

%% Add Noise
ptCloud_P = AddNoise(ptCloud_P, sigma);

%% Add Outliers
ptCloud_P = AddOutliers(ptCloud_P, beta);

%% Show
ptCloud_P.Color = uint8( repmat([0 0 255], ptCloud_P.Count ,1) );
ptCloud_P.Color = uint8( repmat([0 255 0], ptCloud_P.Count, 1) );
pcshow(ptCloud_P)
hold on
pcshow(ptCloud_Q)

%% Save
pcwrite(ptCloud_Q,'ptCloud_Q.pcd','Encoding','ascii');
pcwrite(ptCloud_P,'ptCloud_P.pcd','Encoding','ascii');
SaveTransformationMatrix(T,'trans.mat');