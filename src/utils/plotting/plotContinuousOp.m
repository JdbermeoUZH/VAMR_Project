function [fig] = plotContinuousOp(ldmk_kps_3D, R_C2_W, T_C2_W, img, ...
    ldmk_kps_2D, candidate_kps_2D, fig_num, ...
    estimated_trajectory, ...
    num_landmarks, landmarks_postion, reporting_window, ...
    frame_num, dataset)
    % TODO: Documentation
    
    if (dataset == 0)
        axis_range_x = 50;
        axis_range_y = 50;
        axis_range_z = 50;
        axis_range_z_m = axis_range_z;
    elseif (dataset == 1)
        axis_range_x = 50;
        axis_range_y = 50;
        axis_range_z = 50;
        axis_range_z_m = axis_range_z;
    elseif(dataset == 2)
        axis_range_x = 50;
        axis_range_y = 50;
        axis_range_z = 140;
        axis_range_z_m = 20;
    end
    

    %% Conver everything to homogenous coordinates
    ldmk_kps_3D_h = vertcat(ldmk_kps_3D, ones(1, size(ldmk_kps_3D, 2)));

    % R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]
    % P is a [4xN] matrix containing the triangulated point cloud (in
    % homogeneous coordinates), given by the function linearTriangulation
    ldmk_kps_2D = ldmk_kps_2D.';
    ldmk_kps_2D = [ldmk_kps_2D;ones(1, length(ldmk_kps_2D))];
    candidate_kps_2D = candidate_kps_2D.';
    candidate_kps_2D = [candidate_kps_2D;ones(1, size(candidate_kps_2D, 2))];

    %% Visualize the 3-D landmarks and camera poses
    fig = figure(fig_num);
    subplot(2,3,[1])
    % R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]
    
    % P is a [4xN] matrix containing the triangulated point cloud (in
    % homogeneous coordinates), given by the function linearTriangulation
    plot3(ldmk_kps_3D_h(1,:), ldmk_kps_3D_h(2,:), ldmk_kps_3D_h(3,:), '.');
    hold on;
    % Display camera pose
    
    %plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
    %text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
    
    %center_cam2_W = -R_C2_W'*T_C2_W;
    %plotCoordinateFrame(R_C2_W',center_cam2_W, 0.8);
    %text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
    pose_cam1 = rigid3d(eye(3), zeros(1,3));
    pose_cam2 = rigid3d(inv(R_C2_W), T_C2_W');
    plotCamera('AbsolutePose',pose_cam1,'Opacity',0, 'Color', [0, 1, 0]);
    plotCamera('AbsolutePose',pose_cam2,'Opacity',0, 'Color', [1, 0, 0]);
    
    axis equal
    % center plot on current position
    % axis([T_C2_W(1)-axis_range_x, T_C2_W(1)+axis_range_x, ...
    %    T_C2_W(2)-axis_range_y, T_C2_W(2)+axis_range_y, ...
    %    T_C2_W(3)-axis_range_z_m, T_C2_W(3)+axis_range_z])
    rotate3d on;
    grid
    hold off
    title(sprintf('3D pose of camera and landamarks (@ frame: %.0f)', frame_num))
    
    %% Display image with landmark keypoints and candidate keypoints
    subplot(2,3,[2, 3])
    imshow(img);
    hold on;
    plot(ldmk_kps_2D(1, :), ldmk_kps_2D(2, :), 'g', ...
        'linestyle','none','marker','x', 'MarkerSize', 8, ...
        'DisplayName','Landmark Keypoints');
    plot(candidate_kps_2D(1, :), candidate_kps_2D(2, :), 'r', ...
        'linestyle','none','marker','x', 'MarkerSize', 4, ...
        'DisplayName','Candidate Keypoints');
    legend('Location', 'southoutside','NumColumns',2,'FontSize',5);
    hold off;
    title(sprintf('Frame: %.0f', frame_num));
    

    
    %% Display full trajectory in xz plane
    subplot(2,3,4)
    num_frames = length(num_landmarks(num_landmarks~=0));
    plot(estimated_trajectory(:, end-8), estimated_trajectory(:, end), '--', 'Linewidth', 1);
    xlabel('x');
    ylabel('z');
    axis equal
    %axis([T_C2_W(1)-axis_range, T_C2_W(1)+axis_range, T_C2_W(3)-axis_range, T_C2_W(3)+axis_range])
    title(sprintf('Full Trajectory (@ frame: %.0f)', frame_num));

    %% Display trajectory of last 20 frames with landmarks in xz plane
    subplot(2,3,5)
    plot(ldmk_kps_3D(1, :), ldmk_kps_3D(3, :), '.');
    hold on;
    window_frames_idx = max(num_frames - reporting_window + 1, 1) : num_frames;
    plot(estimated_trajectory(window_frames_idx, end-8), estimated_trajectory(window_frames_idx, end), 'rx', 'Linewidth', 1);
    hold off;
    xlabel('x');
    ylabel('z');
    axis equal
    %axis([T_C2_W(1)-axis_range_x, T_C2_W(1)+axis_range_x, T_C2_W(3)-axis_range_z_m, T_C2_W(3)+axis_range_z])
    title(sprintf('Trajectory of last 20 frames and current landmarks in 2D (@ frame: %.0f)', frame_num));

    %% Display number of tracked landmarks over the last 20 features
    % Plot each point as a horizontal line 
    subplot(2,3,6)
    frames = max(frame_num-reporting_window + 1, frame_num-num_frames + 1) : frame_num;    
    plot(frames, num_landmarks(window_frames_idx));
    title('# of landamarks over last 20 frames')
    
end