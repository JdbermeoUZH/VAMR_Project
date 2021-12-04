function [] = plotBootstrapOutput(P_3D, R, T, img1, img2, matched_keypoints_1, matched_keypoints_2)
    % TODO: Documentation
    % only used for testing if bootstrapping is not giving us
    % shit but usfull stuff

    P_3D_h = vertcat(P_3D, ones(1, length(P_3D)));

    % R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]
    % P is a [4xN] matrix containing the triangulated point cloud (in
    % homogeneous coordinates), given by the function linearTriangulation
    p1 = matched_keypoints_1.';
    p1 = [p1;ones(1, length(p1))];
    p2 = matched_keypoints_2.';
    p2 = [p2;ones(1, length(p2))];
    plotPoseEstimation(P_3D_h, R, T, p1, p2, img1, img2, 1);
end