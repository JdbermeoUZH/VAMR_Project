function [State_now, T_wc_now] = processFrame(Image_now, Image_before, State_before, ...
                                 T_wc_before, hyperparameters, K)
% process frame in continuous operation
% State.P keypoints in current frame -->N*2
% State.X landmarks associated with keypoints -->N*3
% State.C candidate keypoints in current frame -->M*2
% State.F first observation of candidate keypoints -->M*2
% State.T camera pose at the first obervation of keypoints -->M*12
% (vectoriaze the [R|T] in to a 1*12 vector)
% T_wc_before previous camera pose in 3*4

% step1: 4.1 and 4.2
% associate the keypoints to landmarks and estimate the current camera pose

[State_now, T_wc_now, ~] = estimateCurrentPose(Image_now, Image_before,... 
                State_before, T_wc_before, K, hyperparameters);

% step2: 4.3
% triangulate new landmarks

State_now = triangulateNewLandmarks(Image_now, Image_before, State_now,...
              T_wc_now, K, hyperparameters);
  

end
                             