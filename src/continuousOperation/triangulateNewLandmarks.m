function State = triangulateNewLandmarks(Image_now, Image_before, State, T_wc_now, K, hyperparameters)
 %T_wc_now The current camera pose in 3*4

%Dsicard the failed track points and update the candidate keypoints
if size(State.C)>0
    
    point_tracker = vision.PointTracker('MaxBidirectionalError', hyperparameters.klt_MaxBidirectionalError, ...
                                       'NumPyramidLevels', hyperparameters.klt_NumPyramidLevels, ...
                                       'BlockSize', hyperparameters.klt_BlockSize, ...
                                       'MaxIterations', hyperparameters.klt_MaxIterations);

    initialize(point_tracker, State.C, Image_before); 

    
    [candidate_now, validity] = point_tracker(Image_now);

    State.C = candidate_now(validity,:);
    State.F = State.F(validity,:);
    State.T = State.T(validity,:);
end

T_cw_now = invt([T_wc_now;0 0 0 1]);  %world to current camera pose -->4*4
%use this index to remove the candidate_point informtaion that create new
%landmark and fulfill the threshold condition
remove_filter = [];

for i=1:size(State.C,1)
    %wolrd to camera projection matrix of current frame
    Matrix1 = K*T_cw_now(1:3,:); % 3*4
    T_wc_first = reshape(State.T(i,:),3,4);
    T_cw_first = invt([T_wc_first;0 0 0 1]);
     %world to camera projetcion matrix of first frame
    Matrix2 = K*T_cw_first(1:3,:); %3*4
    %get homogeneous coordinate
    point_current = [State.C(i,:),1]'; %3*1
    point_first = [State.F(i,:),1]';   %3*1
    %triangulate to get the new candidate landmarks
    new_landmark = linearTriangulation(point_current, point_first, Matrix1, Matrix2); %4*1
    %compute the angle alpha
    bearing_vector_c = new_landmark(1:3) - T_wc_now(1:3,4);
    bearing_vector_f = new_landmark(1:3) - T_wc_first(1:3,4);
    alpha = acos(dot(bearing_vector_c', bearing_vector_f')/(norm(bearing_vector_c')*...
            norm(bearing_vector_f')));
    %Filter to get the alpha larger than threshold
    if(alpha > hyperparameters.bearing_angle_threshold)
        remove_filter = [remove_filter,i];
        State.X = [State.X;new_landmark(1:3)'];
        State.P = [State.P;State.C(i,:)];   
    end
end
%Remove C,F,T as statement said
State.C(remove_filter,:)=[];
State.F(remove_filter,:)=[];
State.T(remove_filter,:)=[];

%Get the new candidate keypoints in current frame
new_candidate_keypoints = featDetect(Image_now, hyperparameters);

% TODO: Move this line to deatDetect.m as we currently have a custom
% implementation of Harris. 
%new_candidate_keypoints = detectHarrisFeatures(Image_now, ...
%                                'MinQuality', hyperparameters.min_quality, ...
%                                'ROI', hyperparameters.ROI, ...
%                                'FilterSize', hyperparameters.corner_patch_size);

distance_p = pdist2(State.P, new_candidate_keypoints, 'squaredeuclidean', 'Smallest', 1);
if size(State.C)>0
    distance_c = pdist2(State.C, new_candidate_keypoints, 'squaredeuclidean', 'Smallest', 1);
    % Get the candidate keypoints that are far from current keypoints
    new_candidate_keypoints = new_candidate_keypoints(distance_p > hyperparameters.new_candidate_keypoints_dist_thresh... 
                                              & distance_c > hyperparameters.new_candidate_keypoints_dist_thresh,:);
else 
    new_candidate_keypoints = new_candidate_keypoints(distance_p > hyperparameters.new_candidate_keypoints_dist_thresh,:);
end

if size(new_candidate_keypoints,1) ~= 0
    %update the State
    State.C = [State.C; new_candidate_keypoints];
    State.F = [State.F; new_candidate_keypoints];
    vec_T_wc_now = reshape(T_wc_now, 1, 12);
    State.T = cat(1, State.T, repmat(vec_T_wc_now, size(new_candidate_keypoints, 1), 1));
end



end
        
    
    
   