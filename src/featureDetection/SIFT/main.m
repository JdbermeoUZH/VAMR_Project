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

%% hyperparameters
num_scales 			= 3; 			% 3, 	Scales per octave (num octaves is calculated automatically).
sigma 				= 1.6;			% 1,
contrast_threshold	= 0.04;			% 0.04,
rescale_factor 		= 0.3; 			% 0.3,	Rescaling of the original image for speed.

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
	kp = detectSIFTFeatures(img, Sigma=sigma, NumLayersInOctave=num_scales, ContrastThreshold=contrast_threshold);
	[desc{i}, valid_kpts{i}] = extractFeatures(img, kp.Location);
end

%% plot
figure(1);
for i = img_indices
	fprintf('\n\nPlot frame %d\n=====================\n', i);
    img = imread(sprintf('../../../datasets/parking/images/img_%05d.png',i));		% TODO: make me variable
	imshow(img); hold on;
	% plot keypoints (u, v coordinates)
	plot(valid_kpts{i}(:,1), valid_kpts{i}(:,2), 'rx', 'Linewidth', 2);
	% match (skip first frame)
	if (i > 1)
		matches = matchFeatures(desc{i}, desc{i-1}, 'MatchThreshold', 100, 'MaxRatio', 0.7, 'Unique', true);
%		plotMatchesSIFT(matches' ,valid_kpts{i}', valid_kpts{i-1}');	
	end
	hold off;
	pause(0.1);
end
