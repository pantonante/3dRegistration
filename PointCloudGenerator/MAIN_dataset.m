% Description: takes a pre-existing point cloud and generates 
% a dataset of point clouds applying random transformation, and 
% possibly adding noise or outliers. Generate also the descriptor of
% the dataset.
% Author: Pasquale Antonante
% Date: 
% MIT Copyright (c) Pasquale Antonante, Luca Carlone

clear all
close all
clc

addpath('./lib')

rng shuffle

plyFile = 'data/bunny/reconstruction/bun_zipper_res3.ply';
output_folder = '../dataset/bunny_noise';
output_basename = 'bunny';

variable = 'noise'; % possible values {'noise', 'outliers'}
% Note: outliers is expressed as percentage [0,1]
values = 0:0.005:0.1; % noise
%values = 0:0.03:0.7; %outliers

% Random transformation parameters
max_rot = [ 2*pi, 2*pi, 2*pi ];
min_t = [ 0.16 0.16 0.16 ]; % ensure no overlap
max_t = [ 20, 20, 20];

% Downsampling for point cloud P
donwsampling_ratio = 0; % if 0 does not downsample, see downsampling_method
downsampling_method = 'nonuniformGridSample'; % possible values ...
    %... {'nonuniformGridSample', 'gridAverage'}, see `doc pcdownsample`

%% MAIN -- you should not need to edit from now on
delta = 100/length(values); %percentage of the overall proces per pt.cloud

dir=fullfile(pwd,output_folder); % path to the output folder
if exist(dir, 'dir')
   rmdir(dir,'s') % remove it if exists
end
mkdir(dir) %create the output folder
clear dir

fileID = fopen(fullfile(pwd,output_folder,'descriptor.json'),'w');

ptCloud_Q = pcread(plyFile);
disp(['Point Cloud `', plyFile, '` successfully loaded.'])
disp(['Number of points: ', num2str(ptCloud_Q.Count)])

textprogressbar('Generating dataset: ');
fprintf(fileID,'{\n'); 
fprintf(fileID,'\t"dataset": [\n'); 
for i=1:length(values)
    s = values(i);
    % Downsample
    if(donwsampling_ratio>0)
        ptCloud_P = pcdownsample(ptCloud_Q,downsampling_method,donwsampling_ratio);
    else
        ptCloud_P = copy(ptCloud_Q);
    end
    % Apply Random Transformation
    [ptCloud_P, T] = ApplyRandomTransformation(ptCloud_P, ...
        max_rot, min_t, max_t);
    if (strcmpi(variable,'noise'))
        % Add Noise
        ptCloud_P = AddNoise(ptCloud_P, s);
    elseif (strcmpi(variable,'outliers'))
        % Add Outliers
        ptCloud_P = AddOutliers(ptCloud_P, s);
    end
    % Saving
    curr_dir = strrep(char([output_basename,num2str(s)]),'.','');
    mkdir(fullfile(pwd,output_folder,curr_dir))
    ptCloudP_filename = fullfile(pwd,output_folder, curr_dir,'ptCloud_P.pcd');
    ptCloudQ_filename = fullfile(pwd,output_folder, curr_dir,'ptCloud_Q.pcd');
    trans_filename = fullfile(pwd,output_folder, curr_dir,'trans.txt');
    pcwrite(ptCloud_Q,ptCloudQ_filename,'Encoding','binary');
    pcwrite(ptCloud_P,ptCloudP_filename,'Encoding','binary');
    SaveTransformationMatrix(T,trans_filename);
    
    % Json descriptor
    fprintf(fileID,'\t\t{\n');
	fprintf(fileID,'\t\t\t"sigma": "%f",\n',s);
    fprintf(fileID,'\t\t\t"P": "%s",\n',ptCloudP_filename);
    fprintf(fileID,'\t\t\t"Q": "%s",\n',ptCloudQ_filename);
    fprintf(fileID,'\t\t\t"T": "%s"\n',trans_filename);
    if i<length(values)
        delimiter = ',';
    else
        delimiter = '';
    end
    fprintf(fileID,'\t\t}%s\n',delimiter); 
    
    textprogressbar(i*delta);
end
fprintf(fileID,'\t]\n}\n'); 
textprogressbar(' done');
