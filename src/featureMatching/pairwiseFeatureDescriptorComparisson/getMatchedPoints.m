function [matched_keypoints_query,matched_keypoints_database] = getMatchedPoints(...
    query_keypoint_location, database_keypoint_location,...
    query_descriptors, database_descriptors, lambda)
%GETMATCHEDPOINTS Return the location of matched points based on feature
% detection and pairwise comparisson.
%   - Match features descriptors from both frames using pairwise comparisson 
%     with euclidean distance.
%   - Return the location of the matched features in both images

% Match descriptors between the two frames
matches = matchDescriptors(query_descriptors, database_descriptors, lambda);

% Create vectors with positions of pixels matched in both frames
matched_keypoints_query = query_keypoint_location(:, matches > 0).';
matched_keypoints_database = database_keypoint_location(:, matches(matches > 0)).';

end

