plyFile = 'bunny/reconstruction/bun_zipper_res3.ply';
output_folder = 'dataset';
output_basename = 'bunny';

sigma =0:0.01:0.1; % noise
beta = 0; % outliers percentage

% Random transformation parameters
max_rot = [ 2*pi, 2*pi, 2*pi ];
min_t = [ 0.16, 0.16, 0.16 ]; % ensure no overlap
max_t = [ 10, 10, 10];


%% MAIN
dir=fullfile(pwd,output_folder);
if exist(dir, 'dir')
   rmdir(dir,'s')
end
mkdir(dir)
clear dir

fileID = fopen(fullfile(pwd,output_folder,'descriptor.json'),'w');

ptCloud_Q = pcread(plyFile);
disp(['Number of points: ', num2str(ptCloud_Q.Count)])

disp('You can use the following JSON code as part of the dataset part in the experiment file')

fprintf(fileID,'{\n'); 
for i=1:length(sigma)
    s = sigma(i);
    % Apply Random Transformation
    [ptCloud_P, T] = ApplyRandomTransformation(ptCloud_Q, ...
        max_rot, min_t, max_t);
    % Add Noise
    ptCloud_P = AddNoise(ptCloud_P, s);
    % Add Outliers
    %ptCloud_P = AddOutliers(ptCloud_P, beta);
    % Saving
    curr_dir = strrep(char([output_basename,num2str(s)]),'.','');
    mkdir(fullfile(pwd,output_folder,curr_dir))
    ptCloudP_filename = fullfile(pwd,output_folder, curr_dir,'ptCloud_P.pcd');
    ptCloudQ_filename = fullfile(pwd,output_folder, curr_dir,'ptCloud_Q.pcd');
    trans_filename = fullfile(pwd,output_folder, curr_dir,'trans.txt');
    pcwrite(ptCloud_Q,ptCloudQ_filename,'Encoding','ascii');
    pcwrite(ptCloud_P,ptCloudP_filename,'Encoding','ascii');
    SaveTransformationMatrix(T,trans_filename);
    
    % Json descriptor
    fprintf(fileID,'\t{\n');
	fprintf(fileID,'\t\t"sigma": "%f",\n',s);
    fprintf(fileID,'\t\t"P": "%s",\n',ptCloudP_filename);
    fprintf(fileID,'\t\t"Q": "%s",\n',ptCloudQ_filename);
    fprintf(fileID,'\t\t"T": "%s"\n',trans_filename);
    if i<length(sigma)
        delimiter = ',';
    else
        delimiter = '';
    end
    fprintf(fileID,'\t}%s\n',delimiter); 
end
fprintf(fileID,'}\n'); 
