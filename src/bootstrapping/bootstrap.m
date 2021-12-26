function [R, T, P_3D, matched_keypoints_1, matched_keypoints_2] = bootstrap(datasets, hyperparameters)
    % @brief:   depending on the algorithms set in the given hyperparameters we will 
    %           calculate the rotation, translation and a 3D point cloud of matched keypoints
    %           between two given images (from datasets). As a bounus we also return all
    %           matched keypoints (for plotting ... not necessarily needed)
    %
    % @param(datasets)          :   datasets variables loaded from LoadProjectImages
    % @param(hyperparameters)   :   hyperparameters loaded from LoadHyperParams
    %
    % @return(R)                :   rotation matrix between two cameras
    % @return(T)                :   translation vector between two cameras
    % @return(P_3D)             :   3D point cloud of matched points 
    % @return(matched_keypoints_1): matched points in image from camera 1 (with outliers removed)
    % @return(matched_keypoints_2): matched points in image from camera 2 (with outliers removed)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Simplify long var names
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    img0    = datasets.img0;
    img1    = datasets.img1;
    K       = datasets.K;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % could also just use hyperparameters.the_thing_to_use in every function 
    featDetec_algo              = hyperparameters.featDetec_algo;
    ransac_algo                 = hyperparameters.ransac_algo;
    featDetec_matchType         = hyperparameters.featDetec_matchType;

    % Harris Stuff
    corner_patch_size           = hyperparameters.corner_patch_size;
    harris_kappa                = hyperparameters.harris_kappa;
    num_keypoints               = hyperparameters.num_keypoints;
    nonmaximum_supression_radius= hyperparameters.nonmaximum_supression_radius;
    descriptor_radius           = hyperparameters.descriptor_radius;
    match_lambda                = hyperparameters.match_lambda;
    
    % SIFT Stuff
    num_scales                  = hyperparameters.sift_num_scales;
    sigma                       = hyperparameters.sift_sigma;
    contrast_threshold          = hyperparameters.sift_contrast_threshold;
    
    % Pairwise Matching Stuff
    match_threshold             = hyperparameters.match_threshold;
    match_max_ratio             = hyperparameters.match_max_ratio;
    match_unique                = hyperparameters.match_unique;
    
    % KLT Matching Stuff
    NumPyramidLevels            = hyperparameters.klt_NumPyramidLevels;           
    MaxBidirectionalError       = hyperparameters.klt_MaxBidirectionalError;        
    MaxIterations               = hyperparameters.klt_MaxIterations;           
    BlockSize                   = hyperparameters.klt_BlockSize; 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Lets Go %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Find the keypoint correspondences between the two images
    % --> extract keypoints and features (descriptors)
    % img0
    [keypoints_1, descriptors_1] = featDetect(img0, hyperparameters);
    if (featDetec_matchType == "Pairwise")
        % img1
        [keypoints_2, descriptors_2] = featDetect(img1, hyperparameters);
        % Create vectors with positions of pixels matched in both frames
        [matched_keypoints_1, matched_keypoints_2] = getMatchedPoints(...
            keypoints_2, keypoints_1, descriptors_2, descriptors_1, ...
            hyperparameters);

    elseif (featDetec_matchType == "KLT")
        [matched_keypoints_1, matched_keypoints_2, ~] = ...
            getKLTMatches(img0, keypoints_1, img1, ...
                          NumPyramidLevels, MaxBidirectionalError, ...
                          MaxIterations, BlockSize);
    end

    % Estimate the pose change with ransac
    % choose which ransac algo to use
    % TODO: make this a function! Might need ransac stuff more often elsewhere
    if(ransac_algo == "8point")
        %% Estimate the pose change with 8-point ransac
        % Get the fundamental matrix
        % TODO: Come up with a smart number of parameters, particularly for the
        %       number of trials
        [F, inliers, ~] = estimateFundamentalMatrix(...
            matched_keypoints_1, matched_keypoints_2, ...
            'Method','RANSAC',...
            'NumTrials', hyperparameters.eightPointNumTrials, ...
            'DistanceThreshold', hyperparameters.eightPointDistanceThreshold,...
            'Confidence', hyperparameters.eightPointConfidence);

    elseif (ransac_algo == "5point")
        %TODO
    end
    
    matched_keypoints_1 = matched_keypoints_1(inliers>0, :);
    matched_keypoints_2 = matched_keypoints_2(inliers>0, :);
    % Get the essential matrix. We assume K_1=K_2
    [R, T, P_3D] = recoverPoseFromFundamentalMatrix(...
        F, K, K,matched_keypoints_1, matched_keypoints_2);
end