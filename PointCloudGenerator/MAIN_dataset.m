% Description: takes a pre-existing point cloud and generates 
% a dataset of point clouds applying random transformation, and 
% possibly adding noise or outliers. Generate also the descriptor of
% the dataset.
% Author: Pasquale Antonante, Luca Carlone (MIT)
% Date: 
% MIT Copyright (c) Pasquale Antonante, Luca Carlone (MIT)

clear all
close all
clc

rng shuffle

%% Configuration

plyFile_P = 'data/bunny/reconstruction/bun_zipper_res3.ply'; % moving
plyFile_Q = 'data/bunny/reconstruction/bun_zipper_res3.ply'; % fix
output_folder = 'bunny_noise'; % the output folder
output_basename = 'bunny'; %the basename of the folder containing
	% the point clouds associated with the current `value`level
    
dataset_multiplier = 20; % how many times the same noise level (or outlier
    % percentage) should be used?

variable = 'noise'; % possible values {'noise', 'outliers', ''}
% Note: outliers is expressed as percentage [0,1]
values = 0:0.0005:0.005; % noise
%values = 0:0.025:0.5; %outliers

% Random transformation parameters
max_rot = [ 2*pi, 2*pi, 2*pi ];
min_t = 'auto'; % a vector 3x1 or 'auto'
max_t = [ 5, 5, 5]; % if min_t is 'auto' thes are multipliers (i.e. max_t.*min_t)

% Downsampling for point cloud P
donwsampling_ratio = 0; % if 0 does not downsample, see downsampling_method
downsampling_method = 'nonuniformGridSample'; % possible values ...
    %... {'nonuniformGridSample', 'gridAverage'}, see `doc pcdownsample`

%% MAIN -- you should not need to edit from now on
delta = 100/(length(values)*dataset_multiplier); %percentage of the overall proces per pt.cloud

dir=fullfile(pwd, output_folder); % path to the output folder
if exist(dir, 'dir')
   rmdir(dir,'s') % remove it if exists
end
mkdir(dir) %create the output folder
clear dir

% Load the point cloud Q
ptCloud_Q = pcread(plyFile_Q);
disp(['Point Cloud `', plyFile_Q, '` successfully loaded.'])
disp(['Number of points: ', num2str(ptCloud_Q.Count)])
ptCloudQ_filename = fullfile(pwd, output_folder, ...
    'ptCloud_Q.pcd');
savepcd(ptCloudQ_filename, ptCloud_Q.Location', 'binary'); %this pt.cloud never changes

% Initialize the dataset descriptor
dd = DatasetDescriptor(variable, getDiameter(ptCloud_Q));

% Load the point cloud P
ptCloud_P = pcread(plyFile_P);

% Downsample (once for all)
if(donwsampling_ratio>0)
    ptCloud_P = pcdownsample(ptCloud_P, ...
        downsampling_method, donwsampling_ratio);
    disp(['Number of points after downsampling: ', ...
        num2str(ptCloud_P.Count)])
end

% Minimum translation is required, if 'auto' eanbled, it need to be computed
if ischar(min_t) && strcmpi(min_t, 'auto')
    min_t = [ % ensure no overlap
    abs(ptCloud_Q.XLimits(1)-ptCloud_Q.XLimits(2)) ...
    abs(ptCloud_Q.YLimits(1)-ptCloud_Q.YLimits(2))...
    abs(ptCloud_Q.ZLimits(1)-ptCloud_Q.ZLimits(2))];
    max_t = max_t.*min_t;
end

textprogressbar('Generating dataset: ');
for i=1:length(values)
    s = values(i);
    curr_dir = strrep(char([output_basename,num2str(s)]),'.','');
    mkdir(fullfile(pwd, output_folder, curr_dir))
    for j=1:dataset_multiplier
        % Apply Random Transformation
        [ptCloud_TP, T] = ApplyRandomTransformation(ptCloud_P, ...
            max_rot, min_t, max_t);
        if (strcmpi(variable,'noise'))
            % Add Noise
            ptCloud_TP = AddNoise(ptCloud_TP, s);
        elseif (strcmpi(variable,'outliers'))
            % Add Outliers
            ptCloud_TP = AddOutliers(ptCloud_TP, s);
        end
        % Saving        
        ptCloudP_filename = fullfile(pwd, output_folder, curr_dir, ...
            sprintf('ptCloud_P%d.pcd', j));        
        savepcd(ptCloudP_filename,ptCloud_TP.Location','binary')
        trans_filename = fullfile(pwd, output_folder, curr_dir, ...
            sprintf('T%d.txt', j));
        SaveTransformationMatrix(T,trans_filename);
        % Updating DatasetDescriptor
        dd = dd.add_PointClouds(s, ptCloudP_filename,...
            ptCloudQ_filename, trans_filename);
        % Updatinf Progress Bar
        textprogressbar(((i-1)*dataset_multiplier+j)*delta);
    end
end
textprogressbar(' done');

%% Saving Descriptor file
dd.to_JSON(fullfile(pwd, output_folder, 'descriptor.json'))

%% Clean up
clear
