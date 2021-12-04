function [R, T, P_3D] = bootstrap(datasets, hyperparameters)
    % TODO: Documentation
    % simplify long var names
    img0    = datasets.img0;
    img1    = datasets.img1;
    K       = datasets.K;

    featDetec_algo              = hyperparameters.featDetec_algo;
    ransac_algo                 = hyperparameters.ransac_algo;
    corner_patch_size           = hyperparameters.corner_patch_size;
    harris_kappa                = hyperparameters.harris_kappa;
    num_keypoints               = hyperparameters.num_keypoints;
    nonmaximum_supression_radius= hyperparameters.nonmaximum_supression_radius;
    descriptor_radius           = hyperparameters.descriptor_radius;
    match_lambda                = hyperparameters.match_lambda;

    % Find the keypoint correspondences between the two images
    % --> extract keypoints and features (descriptors)
    if (featDetec_algo == "Harris")
        % img0
        [keypoints_1, descriptors_1] = getHarrisFeatures(img0, ...
            corner_patch_size, harris_kappa, num_keypoints, nonmaximum_supression_radius,...
            descriptor_radius);
        % img1
        [keypoints_2, descriptors_2] = getHarrisFeatures(img1, ...
            corner_patch_size, harris_kappa, num_keypoints, nonmaximum_supression_radius,...
            descriptor_radius);
    elseif (featDetec_algo == "SIFT")
        % TODO
    end
    % Create vectors with positions of pixels matched in both frames
    [matched_keypoints_1, matched_keypoints_2] = getMatchedPoints(...
        keypoints_2, keypoints_1, descriptors_2, descriptors_1, match_lambda);

    % Estimate the pose change with ransac
    % choose which ransac algo to use
    if(ransac_algo == "8point")
        %% Estimate the pose change with 8-point ransac
        % Get the fundamental matrix
        % TODO: Come up with a smart number of parameters, particularly for the
        %       number of trials
        [F, inliers, status] = estimateFundamentalMatrix(...
            matched_keypoints_1, matched_keypoints_2, ...
            'Method','RANSAC',...
            'NumTrials',2000,'DistanceThreshold',1e-4);

    elseif (ransac_algo == "5point")
        %TODO
    end

    % Get the essential matrix. We assume K_1=K_2
    [R, T, P_3D] = recoverPoseFromFundamentalMatrix(...
        F, K, K,matched_keypoints_1, matched_keypoints_2);

    % R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]
    % P is a [4xN] matrix containing the triangulated point cloud (in
    % homogeneous coordinates), given by the function linearTriangulation
    plot3(P_3D_h(1,:), P_3D_h(2,:), P_3D_h(3,:), 'o');
    p1 = matched_keypoints_1.';
    p1 = [p1;ones(1, length(p1))];
    p2 = matched_keypoints_2.';
    p2 = [p2;ones(1, length(p2))];
    plotPoseEstimation(P_3D_h, R, T, p1, p2, img0, img1, 1);
end