function [fig_count, matched_keypoints_1, matched_keypoints_2] = continousPoseEstimationTest(datasets, hyperparameters, fig_count)
%CONTINOUSPOSEESTIMATIONTEST Summary of this function goes here
%   Detailed explanation goes here
    %% Initialize variables
    poses = cell(length(datasets.imgs) + 2);

    %% Bootstrap the initial 3D-Point cloud 
    [R, T, P_3D, matched_keypoints_1, matched_keypoints_2] = bootstrap(datasets, hyperparameters);
    
    % TODO: Include set of asserts for dimensions of each of the outputs inside
    % the function.

    % Create the initial state with our second keyframe
    num_landmarks = length(matched_keypoints_2);

    T_wc_before = [R, T];
    img_old = datasets.img1; 
    State_before.X = P_3D.';                     % Mx3 array of 3D coordinates of landamarks/keypoints
    State_before.P = matched_keypoints_2;        % Mx2 array of 2D coordinates of landmarks/keypoints in the keyframe's Image plane
    State_before.C = ones(num_landmarks, 3);     % Mx3 array of 3D coordinates of candidate landamarks/keypoints of non-keyframes
    State_before.F = ones(num_landmarks, 2);     % Mx2 array of 2D coordinates of candidate landmarks/keypoints in the image plane where they were first detected
    State_before.T = ones(num_landmarks, 12);    % Mx12 array of poses of the candidate landmarks/keypoints when they were first located

    poses{1} = [eye(3), zeros(3, 1)];            % Pose of the first keyframe
    poses{2} = T_wc_before;                      % Pose of the seconf keyframe
    
    figure(fig_count);
    %% Continious estimation of points with frames after last keyframe
    for i = 1:length(datasets.imgs)
        % Retrieve new frame
        img_now = datasets.imgs{i};

        % Get the current state
        [State_now, T_wc_now, State_before] = estimateCurrentPose( ...
            img_now, img_old, State_before, T_wc_before, ...
            datasets.K, hyperparameters);

        % Persist the new pose found
        poses{i + 1} = T_wc_now;             % Pose of the i-th frame

        % Plot result of pose estimation and matched features btw frames
        plotBootstrapOutput(State_now.X.', T_wc_now(:, 1:3), T_wc_now(:, 4), ...
            img_old, img_now, State_before.P, State_now.P, fig_count);

        % Set values for next iteration
        img_old = img_now;
        State_before = State_now;
        T_wc_before = T_wc_now;
    end
end
