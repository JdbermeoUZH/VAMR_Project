function [State_now, T_wc_now, State_before] = estimateCurrentPose( ...
    Image_now, Image_before, State_before, T_wc_before, ...
    K, hyperparameters)
    %Implementation of 4.1 and 4.2
    % T_wc_before: 3*4 matrix of previous camera pose w.r.t world
    
    keypoints_before = State_before.P;
    landmarks_before = State_before.X;

    %Using KLT to match the keypoints
    [keypoints_before, keypoints_now, validity] = ...
        getKLTMatches(Image_before, keypoints_before, Image_now, ...
                        hyperparameters.klt_NumPyramidLevels, ...
                        hyperparameters.klt_MaxBidirectionalError, ... 
                        hyperparameters.klt_MaxIterations, ... 
                        hyperparameters.klt_BlockSize);

    landmarks_now = landmarks_before(validity,:); 

    output = getFundamentalMatrix(keypoints_before, keypoints_now, hyperparameters);
    F               = output.F;
    inliersIndex    = output.inliers;
    inliers_before  = output.inlierPts1;
    inliers_now     = output.inlierPts2;
    landmarks_now   = landmarks_now(inliersIndex,:);
    
    %Get the relative pose
    [R, T, P_3D] = recoverPoseFromFundamentalMatrix(F, K, K, inliers_before, inliers_now);
    
    %Get the pose of camera 2 w.r.t world
    T_nb        = [R,T;zeros(1,3),1];   %T_c2_c1
    T_wc_now    = T_wc_before*invt(T_nb); %T_w_c1 * T_c1_c2 --> 3*4
    
    %Set the state
    State_before.P = inliers_before(:,1:2); %-->N*2
    State_now.P = inliers_now(:,1:2);       %-->N*2
    State_now.X = landmarks_now;
    State_now.F = State_before.F;
    State_now.C = State_before.C;
    State_now.T = State_before.T;

end