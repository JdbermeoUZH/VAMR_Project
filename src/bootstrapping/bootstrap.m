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
    featDetec_matchType         = hyperparameters.featDetec_matchType;
    
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
    output = getFundamentalMatrix(matched_keypoints_1, matched_keypoints_2, hyperparameters);
    F               = output.F;
    matchedInliers1 = output.inlierPts1;
    matchedInliers2 = output.inlierPts2;

    % Get the relative rotaion and translatoin between camera frames. We assume K_1=K_2
    [R, T, P_3D] = recoverPoseFromFundamentalMatrix(...
        F, K, K, matchedInliers1, matchedInliers2);
end