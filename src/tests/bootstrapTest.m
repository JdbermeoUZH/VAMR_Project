function [fig_count, matched_keypoints_1, matched_keypoints_2] = bootstrapTest(datasets, hyperparameters, fig_count)
    % actually call bootstrap (without plotting anything because later on we also don't 
    % want to plot everything ... this would just get messy)
    [R, T, P_3D, matched_keypoints_1, matched_keypoints_2] = bootstrap(datasets, hyperparameters);
    % now plot (because here we are testing so we WANT to plot data)
    figure(fig_count);
    plotBootstrapOutput(P_3D, R, T, ...
        datasets.img0, datasets.img1, ...
        matched_keypoints_1, matched_keypoints_2, fig_count);
    fig_count = fig_count + 1;
end
