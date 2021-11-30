function plotMatches(matches, img_prev, img_new, keypoints_prev, keypoints_new)
    % this function takes matched indices (matches), extracted keypoints and 
    % the corresponding images and plots them in an overlapping manner.
    % It also marks the keypoints and connects matched ones.

	matchedPoints_prev  = keypoints_prev(matches(:,1),:);
	matchedPoints_new   = keypoints_new(matches(:,2),:);
	showMatchedFeatures(img_prev, img_new, matchedPoints_prev, matchedPoints_new);

end
