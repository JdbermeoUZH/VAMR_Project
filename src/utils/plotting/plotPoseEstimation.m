function [] = plotPoseEstimation(P_3D, R_C2_W, T_C2_W, p1, p2, img_1, img_2, fig_num)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%% Visualize the 3-D scene
figure(fig_num),
subplot(fig_num,3,1)

% R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]

% P is a [4xN] matrix containing the triangulated point cloud (in
% homogeneous coordinates), given by the function linearTriangulation
plot3(P_3D(1,:), P_3D(2,:), P_3D(3,:), 'o');

% Display camera pose

plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

center_cam2_W = -R_C2_W'*T_C2_W;
plotCoordinateFrame(R_C2_W',center_cam2_W, 0.8);
text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

axis equal
rotate3d on;
grid

% Display matched points
subplot(fig_num,3,2)
imshow(img_1,[]);
hold on
plot(p1(1,:), p1(2,:), 'ys');
title('Image 1')

subplot(fig_num,3,3)
imshow(img_2,[]);
hold on
plot(p2(1,:), p2(2,:), 'ys');
title('Image 2')
end

