function [State_now, T_wc_now, State_before] = estimateCurrentPose( ...
    Image_now, Image_before, State_before, T_wc_before, ...
    K, hyperparameters)
%Implementation of 4.1 and 4.2
% T_wc_before: 3*4 matrix of previous camera pose w.r.t world

%Using KLT to match the keypoints
[keypoints_before, keypoints_now, validity] = getKLTMatches( ...
    Image_before, State_before.P, Image_now, ...
    hyperparameters.klt_NumPyramidLevels, ...
    hyperparameters.klt_MaxBidirectionalError, ...
    hyperparameters.klt_MaxIterations, hyperparameters.klt_BlockSize);

landmarks_now = State_before.X(validity,:); 

if (hyperparameters.poseEstimationAlgo == "8point")
    %8 points algorithms with RANSAC
    [F, inliersIndex] = estimateFundamentalMatrix(... 
        keypoints_before, keypoints_now, ...
        'Method','RANSAC',...
        'NumTrials', hyperparameters.eightPointNumTrials, ...
        'DistanceThreshold', hyperparameters.eightPointDistanceThreshold,...
        'Confidence', 90);%hyperparameters.eightPointConfidence);
    
    %Filter inliers
    inliers_before = keypoints_before(inliersIndex,:);
    inliers_now = keypoints_now(inliersIndex,:);
    landmarks_now = landmarks_now(inliersIndex,:);
    
    %Transform to homogeneous coordinates --> N*3
    inliers_before = [inliers_before, ones(length(inliers_before), 1)];
    inliers_now = [inliers_now, ones(length(inliers_now), 1)]; 
    
    %Get the relative pose
    E = K'* F * K;
    [R,u3] = decomposeEssentialMatrix(E);
    [R,T] = disambiguateRelativePose( ...
        R, u3, inliers_before.', inliers_now.', K, K);
    
    %Get the pose of camera 2 w.r.t world
    T_nb = [R,T;zeros(1,3),1];   %T_c2_c1
    T_wc_now = T_wc_before*invt(T_nb); %T_w_c1 * T_c1_c2 --> 3*4
    
    %Set the state
    State_before.P = inliers_before(:,1:2); %-->N*2
    State_now.P = inliers_now(:,1:2);       %-->N*2
    State_now.X = landmarks_now;

elseif (hyperparameters.poseEstimationAlgo == "P3P")
    disp('Do P3PRansac')
end

end