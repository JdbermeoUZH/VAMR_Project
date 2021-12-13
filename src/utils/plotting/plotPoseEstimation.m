function [] = plotPoseEstimation(P_3D, R_C2_W, T_C2_W, p1, p2, img_1, img_2, fig_num)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%% Visualize the 3-D scene
figure(fig_num),
subplot(1,2,1)

% R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]

% P is a [4xN] matrix containing the triangulated point cloud (in
% homogeneous coordinates), given by the function linearTriangulation
plot3(P_3D(1,:), P_3D(2,:), P_3D(3,:), 'o');
hold on;
% Display camera pose

%plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
%text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

%center_cam2_W = -R_C2_W'*T_C2_W;
%plotCoordinateFrame(R_C2_W',center_cam2_W, 0.8);
%text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
pose_cam1 = rigid3d(eye(3), zeros(1,3));
pose_cam2 = rigid3d(inv(R_C2_W),T_C2_W');
plotCamera('AbsolutePose',pose_cam1,'Opacity',0, 'Color', [1, 0, 0]);
plotCamera('AbsolutePose',pose_cam2,'Opacity',0, 'Color', [0, 0, 1]);

axis equal
rotate3d on;
grid
hold off

% Display matched points
subplot(1,2,2)
% flip because showMatchedFeatures somwhow expects that
matchedPoints_1 = [p1(1,:); p1(2,:)]';
matchedPoints_2 = [p2(1,:); p2(2,:)]';
showMatchedFeatures(img_1, img_2, matchedPoints_1, matchedPoints_2);
end

