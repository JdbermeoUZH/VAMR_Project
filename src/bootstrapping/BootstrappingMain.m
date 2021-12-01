% Notes:
%	- This script calculates a 3D point cloud using 2D-2D method (8-point
%	Ransac) and triangulation 
%   - the imgs{} variable is extracted through the LoadProjectImages script
% 	- hyperparams are set in the LoadHyperParams script
% 	- filepaths are set in the LoadFilePaths script

clc

%% Range of images to use in test run
test					= true;
test_range				= 1;
%% Load the images
LoadProjectImages           % Starting images stored in img0 and img1
                            %  All other frames stored in imgs
%% Find the keypoints between the two images
% Feature Matching using Harris
harris_scores = harris(img0, corner_patch_size, harris_kappa);
harris_scores_2 = harris(img1, corner_patch_size, harris_kappa);

% Select features based on on non-max supression 
keypoints = selectKeypoints(...
    harris_scores, num_keypoints, nonmaximum_supression_radius);
descriptors = describeKeypoints(img0, keypoints, descriptor_radius);

keypoints_2 = selectKeypoints(...
    harris_scores_2, num_keypoints, nonmaximum_supression_radius);

% Describe the points using HOG
descriptors_2 = describeKeypoints(img1, keypoints_2, descriptor_radius);

matches = matchDescriptors(descriptors_2, descriptors, match_lambda);
