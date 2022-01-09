function [State_now, T_wc_now] = estimateCurrentPose( ...
    Image_now, Image_before, State_before, T_wc_before, ...
    K, hyperparameters)
% Implementation of 4.1 and 4.2
% T_wc_before: 3*4 matrix of previous camera pose w.r.t world

% Using KLT to match the keypoints
% force to KLT here
old_matching = hyperparameters.featDetec_matchType;
hyperparameters.featDetec_matchType = "KLT";
% swipes right ... ITS A MATCH!!!
[keypoints_before, keypoints_now, validity] = matchFeat( ...
    Image_before, State_before.P, Image_now, hyperparameters);
% reset matching method
hyperparameters.featDetec_matchType = old_matching;

landmarks_now = State_before.X(validity,:); 
PnPfailed = false;

if (hyperparameters.poseEstimationAlgo == "PnP")
    try
        [R, T, inliersIndex] = estimateWorldCameraPose(keypoints_now, landmarks_now, hyperparameters.CameraParams, 'Confidence', 0.9, 'MaxNumTrials', 5000, 'MaxReprojectionError', 1);
       
        %Filter inliers
        inliers_now = keypoints_now(inliersIndex,:);
        landmarks_now = landmarks_now(inliersIndex,:);
        
        %Get the pose of camera 2 w.r.t world
        T_nb = [R,T.';zeros(1,3),1];   %T_c2_c1
        T_wc_now = T_wc_before*invt(T_nb); %T_w_c1 * T_c1_c2 --> 3*4
        
        %Set the state
        State_now.P = inliers_now(:,1:2);
        State_now.X = landmarks_now;
        State_now.F = State_before.F;
        State_now.C = State_before.C;
        State_now.T = State_before.T;
    catch
        PnPfailed=true;
    end
end


if (hyperparameters.poseEstimationAlgo == "8point" || PnPfailed)
    %8 points algorithms with RANSAC
    try
        output = getFundamentalMatrix(keypoints_before, keypoints_now, hyperparameters);
        F               = output.F;
        inliersIndex    = output.inliers;
        inliers_before  = output.inlierPts1;
        inliers_now     = output.inlierPts2;
        landmarks_now   = landmarks_now(inliersIndex,:);
        
        %Get the relative pose
        [R, T, ~] = recoverPoseFromFundamentalMatrix(F, K, K, inliers_before, inliers_now);
        State_now.P = inliers_now(:,1:2);       %-->N*2
    catch
        warning('Problem using 8-point');
        R = eye(3);
        T = zeros(3,1);
        State_now.P = State_before.P;       %-->N*2
    end
    %Get the pose of camera 2 w.r.t world
    T_nb = [R,T;zeros(1,3),1];   %T_c2_c1
    T_wc_now = T_wc_before*invt(T_nb); %T_w_c1 * T_c1_c2 --> 3*4
    
    %Set the state
    State_now.X = landmarks_now;
    State_now.F = State_before.F;
    State_now.C = State_before.C;
    State_now.T = State_before.T;

end