function [matched_keypoints_1,matched_keypoints_2, isMatched] = getKLTMatches( ...
    img0, keypoints_1, img1, ...
    NumPyramidLevels, MaxBidirectionalError, MaxIterations, BlockSize)
%GETKLTMATCHES Summary of this function goes here
%   Detailed explanation goes here

KLT_tracker = vision.PointTracker( ...
'NumPyramidLevels', NumPyramidLevels, ...
'MaxBidirectionalError', MaxBidirectionalError, ...
'MaxIterations', MaxIterations, ...
'BlockSize', BlockSize);

initialize(KLT_tracker, keypoints_1, img0);
            
% Track the points in the second image
[keypoints_2, isMatched] = KLT_tracker(img1);

matched_keypoints_1 = keypoints_1(isMatched, :);
matched_keypoints_2 = round(keypoints_2(isMatched, :));

end

