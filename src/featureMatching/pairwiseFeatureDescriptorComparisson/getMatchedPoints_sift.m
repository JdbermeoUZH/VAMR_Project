function [matched_keypoints_query,matched_keypoints_database, matches] = getMatchedPoints_sift(keypoints_query, keypoints_database, ...  
                descriptors_query, descriptors_database, match_threshold, match_max_ratio, match_unique)
    % @brief    returns matched keypoint locations in two pictures
    %
    % @param(keypoints_query)
    % @param(keypoints_database)
    % @param(descriptors_query)
    % @param(descriptors_database)      TODO
    % @param(match_threshold)
    % @param(match_max_ratio)
    % @param(match_unique)

    % get matches using matlabs cool functions
    matches                     = matchFeatures(descriptors_query, descriptors_database, 'MatchThreshold', match_threshold, 'MaxRatio', match_max_ratio, 'Unique', match_unique);
    % get keypoint locations of matched points
    matched_keypoints_query     = keypoints_query(matches(:,1),:);
    matched_keypoints_database  = keypoints_database(matches(:,2),:);
end