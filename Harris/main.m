clear all;
close all;

%% Load Images


parking_path = 'parking'; %'C:\Users\User\Documents\School\VAMR\Project\parking'
bootstrap_frames = [1 3];

LoadProjectImages

%% Harris



% Randomly chosen parameters that seem to work well - can you find better
% ones?
corner_patch_size = 9; %9
harris_kappa = 0.12; %0.08           between 0.04 and 0.15 smaller seems better on parking lot. lesson 5 slide 47
num_keypoints = 150; %200
nonmaximum_supression_radius = 10; %8    bigger ==> descriptors are further apart
descriptor_radius = 9; %9
match_lambda = 6; %4        Bigger lambda ==> less selective in matching descriptors

% Ideas to make more recognizable:
% line recognition: if some features are aligned for two consecutive frames, expect them to be for the rest
% only match points if the distance is not too big
% predict feature movement based on position in image (y position for example in parking lot


% Harris
harris_scores = harris(img0, corner_patch_size, harris_kappa);
assert(min(size(harris_scores) == size(harris_scores)));

%Display
figure('Color', 'w');
subplot(2, 2, 1);
imshow(img0);
subplot(2, 2, 2);
imshow(img0);


subplot(2, 2, 4);
imagesc(harris_scores);
title('Harris Scores');
daspect([1 1 1]);

axis off;


%% Part 2 - Select keypoints

keypoints = selectKeypoints(...
    harris_scores, num_keypoints, nonmaximum_supression_radius);
figure(2);
imshow(img0);
hold on;
plot(keypoints(2, :), keypoints(1, :), 'rx', 'Linewidth', 2);

%% Part 3 - Describe keypoints and show 16 strongest keypoint descriptors

descriptors = describeKeypoints(img0, keypoints, descriptor_radius);
figure(3);
for i = 1:16
    subplot(4, 4, i);
    patch_size = 2 * descriptor_radius + 1;
    imagesc(uint8(reshape(descriptors(:,i), [patch_size patch_size])));
    axis equal;
    axis off;
end

%% Part 4 - Match descriptors between first two images
% img1 = imread('../data/000001.png');
harris_scores_2 = harris(img1, corner_patch_size, harris_kappa);
keypoints_2 = selectKeypoints(...
    harris_scores_2, num_keypoints, nonmaximum_supression_radius);
descriptors_2 = describeKeypoints(img1, keypoints_2, descriptor_radius);

matches = matchDescriptors(descriptors_2, descriptors, match_lambda);

figure(4);
imshow(img1);
hold on;
plot(keypoints_2(2, :), keypoints_2(1, :), 'rx', 'Linewidth', 2);
plotMatches(matches, keypoints_2, keypoints);


%% Part 5 - Match descriptors between all images
figure(5);
img_indices = 1:size(album,3);
clear prev_desc
for i = img_indices
    %img = imread(sprintf('../data/%06d.png',i));
    img = album(:,:,i);
    imshow(img); hold on;
    
    scores = harris(img, corner_patch_size, harris_kappa);
    kp = selectKeypoints(...
        scores, num_keypoints, nonmaximum_supression_radius);
    plot(kp(2, :), kp(1, :), 'rx', 'Linewidth', 2);
    
    desc = describeKeypoints(img, kp, descriptor_radius);
    if (exist('prev_desc', 'var'))
        matches = matchDescriptors(desc, prev_desc, match_lambda);
        plotMatches(matches, kp, prev_kp);
    end
    
    prev_kp = kp;
    prev_desc = desc;
    hold off;
    pause(0.01);
end