function State = triangulateNewLandmarks(Image_now, Image_before, State, T_wc_now, K, hyperparameters)
 %T_wc_now The current camera pose in 3*4

%Dsicard the failed track points and update the candidate keypoints
if size(State.C)>0
    [~, candidate_now, validity] = getKLTMatches( ...
    Image_before, State.C, Image_now, ...
    hyperparameters.klt_NumPyramidLevels, ...
    hyperparameters.klt_MaxBidirectionalError, ...
    hyperparameters.klt_MaxIterations, hyperparameters.klt_BlockSize);

    State.C = candidate_now(validity,:);
    State.F = State.F(validity,:);
    State.T = State.T(validity,:);
end

T_cw_now = invt([T_wc_now;0 0 0 1]);  %world to current camera pose -->4*4
%use this index to remove the candidate_point informtaion that create new
%landmark and fulfill the threshold condition
remove_filter = [];

for i=size(State.C,1)
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
    if(alpha > hyperparameters.angle_threshold)
        remove_filter = [remove_filter,i];
        State.X = [State.X;new_landmark'];
        State.P = [State.P;State.C(i,:)];   
    end
end
%Remove C,F,T as statement said
State.C(remove_filter,:)=[];
State.F(remove_filter,:)=[];
State.T(remove_filter,:)=[];

%Get the new candidate keypoints in current frame
new_candidate_keypoints = detectHarrisFeatures(I_curr, ...
                                'MinQuality', hyperparameters.MinQuality, ...
                                'ROI', hyperparameters.ROI, ...
                                'FilterSize', hyperparameters.FilterSize);
%Get the location in the image
new_keypoints_image = new_candidate_keypoints.Location; % #of_new_keypoints * 2

distance_p = pdist2(State.P, new_keypoints_image, 'squaredeuclidean', 'Smallest', 1);
distance_c = pdist2(State.C, new_keypoints_image, 'squaredeuclidean', 'Smallest', 1);
%Get the cancidate keypoints that are far from current keypoints
new_candidate_keypoints = new_keypoints_image(distance_p > hyperparameters.keypoint_threshold... 
                                              & distance_c > hyperparameters.keypoint_threshold,:);

if size(new_candidate_keypoints,1) ~= 0
    %update the State
    State.C = [State.C; new_candidate_keypoints];
    State.F = [State.F; new_candidate_keypoints];
    vec_T_wc_now = reshape(T_wc_now,1,12);
    State.T = cat(1, State.T, repmat(vec_T_wc_now, size(C_new, 1), 1));
end



end
        
    
    
   