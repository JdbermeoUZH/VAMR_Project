function [fig_count] = continousPoseEstimationTest(datasets, hyperparameters, fig_count)
%CONTINOUSPOSEESTIMATIONTEST Summary of this function goes here
%   Detailed explanation goes here
    %% Initialize variables
    poses = zeros(length(datasets.imgs) + 2, 12);

    %% Bootstrap the initial 3D-Point cloud
    T_wc_initial = [eye(3), zeros(3, 1)];
    poses(1,:) = reshape(T_wc_initial.',1,[]);  % Flatten the pose of the first keyframe
    [R, T, P_3D, ~, matchedInlierPts_2] = bootstrap(datasets, hyperparameters);

    % TODO: Modify the bootstrap function so that it also returns the
    %  not-matched keypoints in image 2, as they are candidates for new
    %  keypooints

    %Get the pose of camera 2 w.r.t world (previous pose)
    T_C1_C2 = [R , T ; zeros(1,3),1];
    T_C2_C1 = T_wc_initial * invt(T_C1_C2);
    poses(2,:) = reshape(T_C2_C1.',1,[]);  % Flatten the pose of the second keyframe

    % TODO: Include set of asserts for dimensions of each of the outputs inside
    % the function.

    % Create the initial state with our second keyframe
    num_landmarks = length(matchedInlierPts_2);

    T_wc_before = T_C2_C1;
    img_old = datasets.img1; 
    State_before.X = P_3D.';                     % Mx3 array of 3D coordinates of landamarks/keypoints
    State_before.P = matchedInlierPts_2;         % Mx2 array of 2D coordinates of landmarks/keypoints in the keyframe's Image plane
    State_before.C = ones(num_landmarks, 3);     % Mx3 array of 3D coordinates of candidate landamarks/keypoints of non-keyframes
    State_before.F = ones(num_landmarks, 2);     % Mx2 array of 2D coordinates of candidate landmarks/keypoints in the image plane where they were first detected
    State_before.T = ones(num_landmarks, 12);    % Mx12 array of poses of the candidate landmarks/keypoints when they were first located

    %% Continious estimation of points with frames after last keyframe
    for i = 1:length(datasets.imgs)
        % Retrieve new frame
        img_now = datasets.imgs{i};

        % Get the current state
        [State_now, T_wc_now, State_before] = estimateCurrentPose( ...
            img_now, img_old, State_before, T_wc_before, ...
            datasets.K, hyperparameters);

        % Persist the new pose found
        poses(i + 2,:) = reshape(T_wc_now.',1,[]);  % Flatten the pose of i-th frame

        % Plot result of pose estimation and matched features btw frames
        plotBootstrapOutput(State_now.X.', T_wc_now(:, 1:3), T_wc_now(:, 4), ...
            img_old, img_now, State_before.P, State_now.P, fig_count);

        % Set values for next iteration
        img_old = img_now;
        State_before = State_now;
        T_wc_before = T_wc_now;
    end

    %% Report Error in estimation of trajectory before BA
    % first remove the poses line of the frames that were not included in
    % the first bootstrapping
    fig_count = reportTrajectoryError(poses, datasets.ground_truth, datasets.ds, ...
        'Error Measurements on translation without BA', fig_count);

end

