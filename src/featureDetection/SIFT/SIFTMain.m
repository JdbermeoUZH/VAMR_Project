% Notes:
%	- the imgs{} variable is extracted through the LoadProjectImages script
% 	- hyperparams are set in the LoadHyperParams script
% 	- filepaths are set in the LoadFilePaths script

%% add necesary paths
%addpath('../../utils/');		% some utility functions (mostly plotting)
%addpath('../../setup/');		% project setup (hyperparams, loading images, ...)

clear all;
close all;
LoadFilePaths
LoadHyperParams
LoadProjectImages
clc

%% do we only test? 
test					= true;
test_range				= 30;

%% plotting stuff
pause_time				= 0.1;	% seconds to pause inbetween images displayed

%% get all necessary paths, images and variables
LoadHyperParams
LoadFilePaths
LoadProjectImages

%%%%%%%%%%%% here we actually start %%%%%%%%%%%%%%%%%%%%

%% init variable sizes
kpts	    = cell(1, 2);
desc		= cell(1, 2);
img_indices = 1:size(imgs,2);

%% iterate through images extract keypoints and descriptortrs
for i = img_indices
	fprintf('\n\nSIFT frame %d\n=====================\n', i);
	% run SIFT Algo to get descriptors and keypoint locations
	[kpts{i}, desc{i}] = sift(imgs{i}, sift_num_scales, sift_sigma, sift_contrast_threshold);
end

%% plot
figure(1);
for i = img_indices
	fprintf('\n\nPlot frame %d\n=====================\n', i);
	% match (skip first frame)
	if (i > 1)
		matches = matchFeatures(desc{i-1}, desc{i}, 'MatchThreshold', match_threshold, 'MaxRatio', match_max_ratio, 'Unique', match_unique);
		% plot keypoints that matched
		plotMatches(matches, imgs{i-1}, imgs{i}, kpts{i-1}, kpts{i});
	end
	hold off;
	pause(pause_time);
end

