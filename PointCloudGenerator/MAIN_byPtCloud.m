% Description: takes a pre-existing point cloud and applies 
% a random transformation, possibly adding noise and outliers. 
% Author: Pasquale Antonante
% Date: 
% MIT Copyright (c) Pasquale Antonante, Luca Carlone

clear all
close all
clc

addpath('./lib')

%% Set options:
plyFile = 'data/bunny/reconstruction/bun_zipper_res3.ply';
sigma = 0; % noise
beta = 0; % outliers percentage [0,1]

%% Read PLY
ptCloud_Q = pcread(plyFile);
disp(['Number of points: ', num2str(ptCloud_Q.Count)])

%% apply random tranformation, add noise and outlier and save results
[ptCloud_P,T] = randomlyTransformPtCloud(ptCloud_P, sigma, beta);

%% Save
savepcd('ptCloud_Q.pcd',ptCloud_Q.Location','binary');
savepcd('ptCloud_P.pcd',ptCloud_P.Location','binary');
SaveTransformationMatrix(T,'trans.txt');

%% Show
pcshow(ptCloud_Q);
hold on
pcshow(ptCloud_P);