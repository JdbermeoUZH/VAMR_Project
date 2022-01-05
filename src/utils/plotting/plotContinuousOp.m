function [] = plotContinuousOp(ldmk_kps_3D, R_C2_W, T_C2_W, img, ...
    ldmk_kps_2D, candidate_kps_2D, fig_num, ...
    num_landmarks, landmarks_postion, reporting_window)
    % TODO: Documentation

    %% Conver everything to homogenous coordinates
    ldmk_kps_3D_h = vertcat(ldmk_kps_3D, ones(1, length(ldmk_kps_3D)));

    % R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]
    % P is a [4xN] matrix containing the triangulated point cloud (in
    % homogeneous coordinates), given by the function linearTriangulation
    ldmk_kps_2D = ldmk_kps_2D.';
    ldmk_kps_2D = [ldmk_kps_2D;ones(1, length(ldmk_kps_2D))];
    candidate_kps_2D = candidate_kps_2D.';
    candidate_kps_2D = [candidate_kps_2D;ones(1, length(candidate_kps_2D))];

    %% Visualize the 3-D landmarks and camera poses
    figure(fig_num),
    subplot(1,2,1)
    
    % R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]
    
    % P is a [4xN] matrix containing the triangulated point cloud (in
    % homogeneous coordinates), given by the function linearTriangulation
    plot3(ldmk_kps_3D_h(1,:), ldmk_kps_3D_h(2,:), ldmk_kps_3D_h(3,:), 'o');
    hold on;
    % Display camera pose
    
    %plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
    %text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
    
    %center_cam2_W = -R_C2_W'*T_C2_W;
    %plotCoordinateFrame(R_C2_W',center_cam2_W, 0.8);
    %text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
    pose_cam1 = rigid3d(eye(3), zeros(1,3));
    pose_cam2 = rigid3d(inv(R_C2_W), T_C2_W');
    plotCamera('AbsolutePose',pose_cam1,'Opacity',0, 'Color', [1, 0, 0]);
    plotCamera('AbsolutePose',pose_cam2,'Opacity',0, 'Color', [0, 0, 1]);
    
    axis equal
    rotate3d on;
    grid
    hold off
    
    %% Display image with landmark keypoints and candidate keypoints
    subplot(1,2,2)
    imshow(img);
    hold on;
    plot(ldmk_kps_2D(1, :), ldmk_kps_2D(2, :), 'g', ...
        'linestyle','none','marker','x', 'MarkerSize', 8);
    plot(candidate_kps_2D(1, :), candidate_kps_2D(2, :), 'r', ...
        'linestyle','none','marker','x', 'MarkerSize', 4);
    hold off;
    
    %% Display number of tracked landmarks over the last 20 features
    % Plot each point as a horizontal line 
    % plot(p_W_frames(3, :), -p_W_frames(1, :), 'rx', 'Linewidth', 3);

    %% Display full trajectory in xz plane
    %plot(p_W_frames(3, :), -p_W_frames(1, :), 'rx', 'Linewidth', 3);

    %% Display trajectory of last 20 frames with landmarks in xz plane
    %plot(p_W_landmarks(3, :), -p_W_landmarks(1, :), '.');
    %hold on;
    %plot(p_W_frames(3, :), -p_W_frames(1, :), 'rx', 'Linewidth', 3);
    %hold off;
end