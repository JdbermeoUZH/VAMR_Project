clear all
clc

%% add necesary paths
addpath('../../utils/');

%% dataset loading
parking_path		= '../../../datasets/parking';	% change to your dataset location
bootstrap_frames	= [1 3];
test				= true;
test_range			= 100;
LoadProjectImages

%% hyperparameters SIFT
sift_num_scales 		= 3; 			% 3, 	Scales per octave (num octaves is calculated automatically).
sift_sigma 				= 1.6;			% 1,
sift_contrast_threshold	= 0.04;			% 0.04,

%% hyperparameters matching
match_threshold		= 100;			% 100
match_max_ratio		= 0.7;			% 0.7
match_unique		= true;			% true

%% init variable sizes
valid_kpts	= cell(1, 2);
desc		= cell(1, 2);

%% iterate through images extract keypoints and descriptortrs
img_indices = 1:size(album,3);
for i = img_indices
	fprintf('\n\nSIFT frame %d\n=====================\n', i);
	%img = album(:,:,i);  % why does this not worK???? 
    img = im2uint8(rgb2gray(imread(sprintf('../../../datasets/parking/images/img_%05d.png',i))));		% TODO: make me variable
	% run SIFT Algo to get descriptors and keypoint locations
	[kpts{i}, desc{i}] = sift(img, sift_num_scales, sift_sigma, sift_contrast_threshold);
end

%% plot
figure(1);
for i = img_indices
	fprintf('\n\nPlot frame %d\n=====================\n', i);
    img = imread(sprintf('../../../datasets/parking/images/img_%05d.png',i));		% TODO: make me variable
	imshow(img); hold on;
	% plot keypoints (u, v coordinates)
	plot(kpts{i}(:,1), kpts{i}(:,2), 'rx', 'Linewidth', 2);
	% match (skip first frame)
	if (i > 1)
		matches = matchFeatures(desc{i}, desc{i-1}, 'MatchThreshold', match_threshold, 'MaxRatio', match_max_ratio, 'Unique', match_unique);
		plotMatchesSIFT(matches' ,kpts{i}', kpts{i-1}');	
	end
	hold off;
	pause(0.1);
end
