function [R, T, P_3D, matchedInliers1, matchedInliers2] = bootstrap(datasets, hyperparameters)
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
    % @return(matched_keypoints_1): matched (inlier) points in image from camera 1 (with outliers removed)
    % @return(matched_keypoints_2): matched (inlier) points in image from camera 2 (with outliers removed)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Simplify long var names
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    img0    = datasets.img0;
    img1    = datasets.img1;
    K       = datasets.K;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Lets Go %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Find the keypoint correspondences between the two images
    % --> extract keypoints and features (descriptors)
    % img0
    [kpts0, desc0] = featDetect(img0, hyperparameters);
    [matched_keypoints_0, matched_keypoints_1, validity] = matchFeat(img0, kpts0, img1, hyperparameters, desc0);

    % Estimate the pose change with ransac
    output = getFundamentalMatrix(matched_keypoints_0, matched_keypoints_1, hyperparameters);
    F               = output.F;
    matchedInliers1 = output.inlierPts1;
    matchedInliers2 = output.inlierPts2;

    % Get the relative rotaion and translatoin between camera frames. We assume K_1=K_2
    [R, T, P_3D] = recoverPoseFromFundamentalMatrix(...
        F, K, K, matchedInliers1, matchedInliers2);
    
    %% Bundle Adjustment for the intial 3D point cloud
    pointTrackArray = [];
    for i = 1:size(matchedInliers1, 1)
        pointTrackArray = [pointTrackArray, pointTrack([1,2], [matchedInliers1(i,:); matchedInliers2(i,:)])];
    end

    % Get estimated absolut pose wrt to world view
    T_relative = [R , T ; zeros(1,3),1];
    T_W= [eye(3) , [0;0;0] ; zeros(1,3),1] * invt(T_relative);

    vSet = viewSet;
    vSet = addView(vSet,1,'Orientation',eye(3),'Location',[0,0,0], 'Points', matchedInliers1);
    vSet = addView(vSet,2,'Orientation',T_W(1:3, 1:3),'Location', T_W(1:3, 4).', 'Points', matchedInliers2);
    cameraPoses = poses(vSet);

    [RefinedP_3D, refinedPoses] = bundleAdjustment( ...
        P_3D.', pointTrackArray,cameraPoses,hyperparameters.CameraIntrinsics, ...
        'FixedViewIDs', [1], 'Verbose', false, 'AbsoluteTolerance', 0.001); 

end