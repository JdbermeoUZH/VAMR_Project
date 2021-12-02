function [keypoints,descriptors] = getHarrisFeatures(...
    img0, corner_patch_size, harris_kappa, ...
    num_keypoints, nonmaximum_supression_radius, ...
    descriptor_radius)
%GETHARRISFEATURES Returns the location and description of harris features
%   Returns the locations of the indentified features
%   Returns the description of said features. Up to now they are ImagePatch
%    descriptors. We could change them to HOG or binary circle one.

% Feature Matching using Harris
harris_scores = harris(img0, corner_patch_size, harris_kappa);

% Select features based on on non-max supression 
keypoints = selectKeypoints(...
    harris_scores, num_keypoints, nonmaximum_supression_radius);

% Describe the points using Image Patch Descriptor (intensities of patch)
descriptors = describeKeypoints(img0, keypoints, descriptor_radius);
end

