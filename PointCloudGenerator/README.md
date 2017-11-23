# Point Cloud Generator

## Introduction
A set of Matlab scripts to generate point clouds with noise and outliers. The purpose of the scripts are:

1. Apply a random rigid transformation
2. Add noise
3. Add outliers
4. Save the point cloud (eventually the dataset)

## Dependencies

* Matlab R2015b
* Computer Vision Toolbox

## Usage

The aim of this mini library is to generate two or more point clouds to be aliged along with the ground truth; there are three example of usage

1. generate two point clouds from a function describing a surface
2. generate two point clouds starting from a pre-existing point cloud
3. generate a dataset of point clouds starting from a pre existing point cloud

#### Generate point cloud from a function
To generate a point cloud from an explicit 3D function, run the script `MAIN_byFunction`. Eventually, modify the first few lines (as below) according to your preferences.

```matlab
%% Point Cloud Generation
% Sombrero function
f = @(X,Y) (10*sin(sqrt(X.^2 + Y.^2)) + 0.1) ./ sqrt(X.^2 + Y.^2);
x_space = -8:.25:8;
y_space = -8:.25:8;
sigma = 0.05; % noise
beta = 3/100; % outliers percentage
```

The result of the script are generate two point clouds, `ptCloud_P.pcd` and `ptCloud_Q.pcd` along with `trans.txt` which contains the random transformation applied to point cloud Q.

#### Generate point cloud from an existing one

Sometimes it is necessary to take a pre-existing point cloud and add noise and outliers. The script`MAIN_byPLY` does exactly this. As before, modify the first few lines (as below) according to your preferences.

```matlab
plyFile = 'bunny/reconstruction/bun_zipper_res3.ply';
sigma = 0.03; % noise
beta = 5/100; % outliers percentage
```

#### Generate a dataset

In this repository you can find the Experiment python script; it is very useful when you want do describe compactly and than execute on a batch of point clouds. The script `MAIN_dataset` generates a bunch of point clouds with noise, outliers and random transformation and generate the relative descriptor used from the python script.

To use it change the first lines of code according to your preferences:

```matlab
plyFile = 'data/bunny/reconstruction/bun_zipper_res3.ply'; %input dataset
output_folder = 'bunny_noise'; % the output folder
output_basename = 'bunny'; %the basename of the folder containing
	% the point clouds associated with the current `value`level

dataset_multiplier = 20; % how many times the same noise level (or outlier
    % percentage) should be used (experimenter will average results on them)?

variable = 'noise'; % for example 'noise', 'outliers', etc.
% Note: outliers is expressed as percentage [0,1]
values = 0:0.0005:0.005; % noise
%values = 0:0.025:0.5; %outliers

% Random transformation parameters
max_rot = [ 2*pi, 2*pi, 2*pi ];
min_t = 'auto'; % a 3x1 vector or `auto` (auto will ensure no overlap)
max_t = [ 5, 5, 5]; % if min_t is `auto` thes are multipliers (i.e. max_t.*min_t)

% Downsampling for point cloud P
donwsampling_ratio = 0; % if 0 does not downsample, see downsampling_method
downsampling_method = 'nonuniformGridSample'; % possible values ...
    %... {'nonuniformGridSample', 'gridAverage'}, see `doc pcdownsample`
```

The output folder will contain the `descriptor.json` file to be used for the experiment descriptor.

## License
MIT Copyright (c) Pasquale Antonante