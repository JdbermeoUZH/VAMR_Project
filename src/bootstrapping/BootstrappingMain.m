% Notes:
%	- This script calculates a 3D point cloud using 2D-2D method (8-point
%	Ransac) and triangulation 
%   - the imgs{} variable is extracted through the LoadProjectImages script
% 	- hyperparams are set in the LoadHyperParams script
% 	- filepaths are set in the LoadFilePaths script

clear all;
close all;
LoadFilePaths
LoadHyperParams
clc

%% Range of images to use in test run
test					= true;
test_range				= 1;

%% Load the images
LoadProjectImages           % Starting images stored in img0 and img1
                            %  All other frames stored in imgs

%% Find the keypoint correspondences between the two images
% TODO: Change this for a function that calls a random feature matcher
%       between both images
% Feature Matching using Harris
[keypoints_1, descriptors_1] = getHarrisFeatures(img0, ...
    corner_patch_size, harris_kappa, num_keypoints, nonmaximum_supression_radius,...
    descriptor_radius);

[keypoints_2, descriptors_2] = getHarrisFeatures(img1, ...
    corner_patch_size, harris_kappa, num_keypoints, nonmaximum_supression_radius,...
    descriptor_radius);

% Create vectors with positions of pixels matched in both frames
[matched_keypoints_1, matched_keypoints_2] = getMatchedPoints(...
    keypoints_2, keypoints_1, descriptors_2, descriptors_1, match_lambda);

%% Estimate the pose change with 8-point ransac
% Get the fundamental matrix
% TODO: Come up with a smart number of parameters, particularly for the
%       number of trials
[F, inliers, status] = estimateFundamentalMatrix(...
    matched_keypoints_1, matched_keypoints_2, ...
    'Method','RANSAC',...
    'NumTrials',2000,'DistanceThreshold',1e-4);

% Get the essential matrix. We assume K_1=K_2
[R, T, P_3D] = recoverPoseFromFundamentalMatrix(...
    F, K, K,matched_keypoints_1, matched_keypoints_2);

%% TODO: Refine estimate of the 3D point-cloud with Bundle Correction

%% Visualize the 3-D scene
figure(1),
subplot(1,3,1)
P_3D_h = vertcat(P_3D, ones(1, length(P_3D)));

% R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]

% P is a [4xN] matrix containing the triangulated point cloud (in
% homogeneous coordinates), given by the function linearTriangulation
plot3(P_3D_h(1,:), P_3D_h(2,:), P_3D_h(3,:), 'o');
p1 = matched_keypoints_1.';
p1 = [p1;ones(1, length(p1))];
p2 = matched_keypoints_2.';
p2 = [p2;ones(1, length(p2))];
plotPoseEstimation(P_3D_h, R, T, p1, p2, img0, img1);