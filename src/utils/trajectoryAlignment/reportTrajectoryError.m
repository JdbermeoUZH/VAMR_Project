function [fig_count] = reportTrajectoryError(poses, poses_ground_truth, dataset, ...
    figure_grid_title, hyperparameters, fig_count)
%REPORTTRAJECTORYERROR Summary of this function goes here
%   Detailed explanation goes here
    
    %% Report ATE 
    estimated_xyz = poses(:, [end-8 end-4 end]).';  %x, y, z coordinates of estimated pose
    num_poses = size(estimated_xyz, 2);
    estimated_xyz = [estimated_xyz(1,:); zeros(1, num_poses); estimated_xyz(3,:)]; % Only compare pose estiamtion in xz plane

    % First we need to define what is the ground truth and which part of the
    % estimated trajectory are we going to compare to
    if(dataset == 2 || dataset == 0) % Kitti or Parking datasets    
        ground_truth_xyz = poses_ground_truth(1:num_poses, :);
        ground_truth_xyz = [ground_truth_xyz(:,1), zeros(num_poses, 1), ground_truth_xyz(:,2)].';
   
    elseif(dataset == 1) % Malaga dataset 
        pose_to_comapre_idx = [1, 106-hyperparameters.bootstrap_frames(2): 106 : num_poses];
        estimated_xyz = estimated_xyz(:, pose_to_comapre_idx);
        num_poses = size(estimated_xyz, 2);
        ground_truth_xyz = poses_ground_truth(1:size(estimated_xyz, 2), :);
        ground_truth_xyz = [ground_truth_xyz(:,1), zeros(num_poses, 1), ground_truth_xyz(:,2)].';
    end

    % Decide whether or not to estimate scale for each dataset
    if(dataset == 0) % Kitti dataset
        estimate_scale = false;
    elseif(dataset == 1) % Malaga dataset 
        estimate_scale = true;
    elseif(dataset == 2)
        estimate_scale = true;
    end

    % Align using Umemaya's algorithm
    estimated_xyz_aligned = umeyama(estimated_xyz, ground_truth_xyz, false, false);
    
    % Calculate ATE
    ATE = sqrt(mean((estimated_xyz_aligned - ground_truth_xyz).^2, 'all'));

    % Plot completely aligned estimated and groundtruth trajectories
    %  along with the ATE
    fig_count = fig_count + 1;
    figure(fig_count);
    subplot(1,2,1)
    y_lim = ceil(max([abs(estimated_xyz_aligned(3, :)) abs(ground_truth_xyz(3, :))]));
    plot(estimated_xyz_aligned(1, :), estimated_xyz_aligned(3, :), '-o', ...
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
    title('Absolute Trajectory Error');
    xlabel('x');
    ylabel('z');

    %% Report RTE 
    % Calcuate RTE: Calculate ATE for multiple cuts of the trajectories
    rte = cell(length(hyperparameters.subtrajectory_lengths), 2);

    for i = 1:length(hyperparameters.subtrajectory_lengths)
        d_frames = hyperparameters.subtrajectory_lengths(i);
        subtrajectory_cuts = 1 : d_frames : length(estimated_xyz);

        subtrajectory_ates = zeros(length(subtrajectory_cuts), 1);
        for j = 1:length(subtrajectory_cuts) - 1 
            start_frame = subtrajectory_cuts(j);
            end_frame = subtrajectory_cuts(j + 1);
            est_subtraj = estimated_xyz(:, start_frame : end_frame); 
            gt_subtraj = ground_truth_xyz(:, start_frame : end_frame);
            aligned_subtraj = umeyama(est_subtraj, gt_subtraj, estimate_scale, false);

            subtrajectory_ates(j) = sqrt(mean(( ...
                aligned_subtraj - gt_subtraj).^2, 'all'));
            
        end
        
        rte{i, 1} = subtrajectory_ates;
        rte{i, 2} = repmat(d_frames, length(subtrajectory_ates),1);
    end

    % Create Boxplot for each subtrajectory length chosen
    subplot(1,2,2)
    flat_rte = cell2mat(rte);
    boxchart(flat_rte(:, 2), flat_rte(:, 1), ...
        "BoxFaceColor",'black', 'MarkerStyle','none', 'BoxFaceAlpha',0, ...
        "LineWidth", 1.25);
    hold on;
    scatter(flat_rte(:, 2), flat_rte(:, 1), 24, "blue", ...
        'filled','MarkerFaceAlpha',0.25,'jitter','on','jitterAmount',0.15); 
    hold off;
    title('Relative Trajectory Error');
    xlabel('Length of sub-trajectory (in frames)');
    ylabel('ATE of sub-trajectories');
    
    sgtitle(figure_grid_title)
end

