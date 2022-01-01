function [fig_count] = reportTrajectoryError(poses, poses_ground_truth, fig_count)
%REPORTTRAJECTORYERROR Summary of this function goes here
%   Detailed explanation goes here
    hold off
    
    %% Report ATE 
    % First we need to align the estimated and ground truth trajectories
    estimated_xyz = poses(:, [end-8 end-4 end]).';  %x, y, z coordinates of estimated pose
    ground_truth_xyz = poses_ground_truth(1:length(estimated_xyz), :);
    ground_truth_xyz = [ground_truth_xyz, zeros(length(ground_truth_xyz), 1)].';
    estimated_xyz = alignEstimateToGroundTruth(ground_truth_xyz, estimated_xyz);

    % Calculate ATE
    ATE = sqrt(mean((estimated_xyz - ground_truth_xyz).^2, 'all'));

    % Plot completely aligned estimated and groundtruth trajectories
    %  along with the ATE
    fig_count = fig_count + 1;
    figure(fig_count);
    subplot(1,2,1)
    
    y_lim = ceil(max([abs(estimated_xyz(3, :)) abs(ground_truth_xyz(3, :))]));
    plot(estimated_xyz(1, :), estimated_xyz(3, :), '-o', ...
        'DisplayName', 'Estimated trajectory (aligned)');
    axis([-inf inf -y_lim y_lim]);
    hold on;
    
    plot(ground_truth_xyz(1, :), ground_truth_xyz(3, :), '-x', ...
        'DisplayName', 'Ground truth trajectory');
    axis([-inf inf -y_lim y_lim]);
    hold off;
    alpha(.5);
    lgd = legend;
    %lgd.FontSize = 14;
    lgd.Title.String = sprintf('ATE: %.2f', ATE);

    %% Report ATE 
    % Calcuate RTE: Calculate ATE for multiple cuts of the trajectories

    % Violin plot for RTE
    subplot(1,2,2)
end

