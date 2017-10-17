% Description: takes a pre-existing point cloud and applies 
% a random transformation, possibly adding noise and outliers. 
% Author: Pasquale Antonante
% Date: 
% MIT Copyright (c) Pasquale Antonante

clear all
close all
clc

addpath('./lib')

%% Set options:
plyFile = 'bunny/reconstruction/bun_zipper_res3.ply';
sigma = 0.05; % noise
beta = 0; % outliers percentage [0,1]

%% Read PLY
ptCloud_Q = pcread(plyFile);
disp(['Number of points: ', num2str(ptCloud_Q.Count)])

%% apply random tranformation, add noise and outlier and save results
[ptCloud_P,T] = randomlyTransformPtCloud(ptCloud_Q,sigma,beta);

%% Save
pcwrite(ptCloud_Q,'ptCloud_Q.pcd','Encoding','ascii');
pcwrite(ptCloud_P,'ptCloud_P.pcd','Encoding','ascii');
SaveTransformationMatrix(T,'trans.txt');