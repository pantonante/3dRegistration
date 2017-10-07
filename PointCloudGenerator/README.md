# Point Cloud Generator

## Introduction
A set of Matlab scripts to generate point clouds with noise and outliers. The purpose of the scripts are:

1. Apply a random rigid transformation
* Add noise
* Add outliers

## Dependencies

* Matlab R2015b
* Computer Vision Toolbox

## Usage

There are two possible entry points for the generation, `MAIN_byFunction` or `MAIN_byPLY`. Both scripts generate due point clouds, `ptCloud_P.pcd` and `ptCloud_Q.pcd` along with `trans.txt` which contains a random transformation applied to point cloud Q.

####Generate point cloud from a function
To generate a point cloud from an explicit 3D function run the script `MAIN_byFunction`. Modify the first few lines (as below) according to your preferences.

```matlab
% Sombrero function
f = @(X,Y) (10*sin(sqrt(X.^2 + Y.^2)) + 0.1) ./ sqrt(X.^2 + Y.^2);
x_space = -8:.25:8;
y_space = -8:.25:8;
sigma = 0.005; % noise
beta = 0.003; % outliers percentage
```

####Generate point cloud from an existing one
Sometimes it is necessary to take a pre-existing point cloud and add noise and outliers. This `MAIN_byPLY` script does exactly this.  As before, modify the first few lines (as below) according to your preferences.

```matlab
plyFile = 'bunny/reconstruction/bun_zipper_res3.ply';
sigma = 0.015; % noise
beta = 0.007; % outliers percentage
```

##License
MIT Copyright (c) Pasquale Antonante