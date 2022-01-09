function [fig_count] = continousPoseEstimationTest(datasets, hyperparameters, fig_count)
%CONTINOUSPOSEESTIMATIONTEST Summary of this function goes here
%   Detailed explanation goes here
    %% Initialize variables
    poses = zeros(length(datasets.imgs), 12);
    hyperparameters.CameraParams = cameraParameters('IntrinsicMatrix', datasets.K);
    principalPoint = [datasets.K(1,3), datasets.K(2,3)];
    focalLength   = [datasets.K(1,1), datasets.K(2,2)];
    imageSize      = size(datasets.img0);
    hyperparameters.CameraIntrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize); 
    
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
    

    T_wc_before = T_C2_C1;
    img_old = datasets.img1;

    State_before.X = P_3D.';                     % Mx3 array of 3D coordinates of landamarks/keypoints
    State_before.P = matchedInlierPts_2;         % Mx2 array of 2D coordinates of landmarks/keypoints in the keyframe's Image plane
    State_before.C = [];     % Mx3 array of 3D coordinates of candidate landamarks/keypoints of non-keyframes
    State_before.F = [];     % Mx2 array of 2D coordinates of candidate landmarks/keypoints in the image plane where they were first detected
    State_before.T = [];    % Mx12 array of poses of the candidate landmarks/keypoints when they were first located

    num_landmarks = zeros(length(datasets.imgs) + 1);
    num_landmarks(1) = size(State_before.X, 1);

    % set up video save
    filename = hyperparameters.featDetec_algo + "_" + datasets.ds;
    if(hyperparameters.test)
        filename = "../output_test/" + filename + "_test";
    else 
        filename = "../output/" + filename;
    end

    writerObj = VideoWriter(filename);  %// create video file
    writerObj.FrameRate = 2;            %// set to 2 frames per second
    open(writerObj);                    %// open file for writing video data

    % measure needed time  
    tStart = tic;
    %% Continious estimation of points with frames after last keyframe
    for i = 1:length(datasets.imgs)
        % Retrieve new frame
        img_now = datasets.imgs{i};

        % Get the current state
        [State_now, T_wc_now] = processFrame( ...
            img_now, img_old, State_before, T_wc_before, ...
            datasets.K, hyperparameters);
            
        % Persist the new pose found
        poses(i + 2,:) = reshape(T_wc_now.',1,[]);  % Flatten the pose of i-th frame

        % Store values for reporting
        num_landmarks(i) = size(State_now.X, 1);
        
        % Plot result of pose estimation and matched features btw frames
        fig = plotContinuousOp(State_now.X.', T_wc_now(:, 1:3), T_wc_now(:, 4), ...
            img_now, State_now.P, State_now.C, fig_count, poses(3 : i + 2, :), ...
            num_landmarks(1:i), State_now.X, hyperparameters.reporting_window, ...
            i + hyperparameters.bootstrap_frames(2), datasets.ds);
        % save plot as movie/gif
        frame = getframe(fig);          %// Capture axes or figure as movie frame
        writeVideo(writerObj,frame);    %// Write video data to file

        % Set values for next iteration
        img_old = img_now;
        State_before = State_now;
        T_wc_before = T_wc_now;
    end
    close(writerObj);                   %// Close video file
    % write needed time to text filek
    tStop           = toc(tStart);
    filename_time   = filename + "_time.txt";
    fid             = fopen(filename_time,'wt');
    fprintf(fid, string(tStop));
    fclose(fid);


    %% Report Error in estimation of trajectory before BA
    % first remove the poses line of the frames that were not included in
    % the first bootstrapping
    filename_error = filename + "_error.png";
    [fig_count, fig] = reportTrajectoryError(poses, datasets.ground_truth, datasets.ds, ...
        'Error Measurements on translation without BA', hyperparameters, fig_count);
    saveas(fig, filename_error);
end

