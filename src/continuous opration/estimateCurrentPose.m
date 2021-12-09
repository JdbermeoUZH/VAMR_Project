function [State_now, T_wc_now] = estimateCurrentPose(Image_now, Image_before, State_before, T_wc_before)

%Using KLT to match the keypoints
KLTtracker = vision.PointTracker();
initialize(KLTtracker, State_before.P, Image_before);  
[points, validaty] = KLTtracker(Image_now);

%Filter to get the valid points
keypoints_before = State_before.P(validaty,:);
keypoints_now = points(validaty,:);
landmarks_now = State_before.X(validaty,:);

%8 points algorithms with RANSAC
[F, inliersIndex] = estimateFundamentalMatrix(keypoints_before, ... 
    keypoints_now,'Method','RANSAC',...
   'NumTrials',30000,'DistanceThreshold',0.01,'Confidence', 90);

%Filter inliers
inliers_before = keypoints_before(inliersIndex,:);
inliers_now = keypoints_now(inliersIndex,:);
landmarks_now = landmarks_now(inliersIndex,:);

%Transform to homogeneous coordinates
inliers_before = [inliers_before;ones(1,lenghth(inliers_before))];
inliers_now = [inliers_now;ones(1,lenghth(inliers_now))]; 

%Get the relative pose
E = K'* F * K;
[R,u3] = decomposeEssentialMatrix(E);
[R,T] = disambiguateRelativePose(R,u3,inliers_before,inliers_now,K,K);

%Get the pose of camera 2 w.r.t world
T_nb = [R,T;zeros(1,3),1];
T_wc_now = T_wc_before*invt(T_nb);

%Set the state
State_now.P = inliers_now;
State_now.X = landmarks_now;

end