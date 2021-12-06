function [] = siftTest(datasets, hyperparameters, fig_count)
    %% plotting stuff
    pause_time				= 0.1;	% seconds to pause inbetween images displayed
    %% init variable sizes
    kpts	    = cell(1, 2);
    desc		= cell(1, 2);
    img_indices = 1:size(datasets.imgs,2);

    %% iterate through images extract keypoints and descriptortrs
    for i = img_indices
    	fprintf('\n\nSIFT frame %d\n=====================\n', i);
    	% run SIFT Algo to get descriptors and keypoint locations
    	[kpts{i}, desc{i}] = sift(datasets.imgs{i}, hyperparameters.sift_num_scales, ...
            hyperparameters.sift_sigma, hyperparameters.sift_contrast_threshold);
    end

    %% plot
    figure(fig_count);
    for i = img_indices
    	fprintf('\n\nPlot frame %d\n=====================\n', i);
    	% match (skip first frame)
    	if (i > 1)
    		matches = matchFeatures(desc{i-1}, desc{i}, ...
                'MatchThreshold',   hyperparameters.match_threshold, ... 
                'MaxRatio',         hyperparameters.match_max_ratio, ... 
                'Unique',           hyperparameters.match_unique);
    		% plot keypoints that matched
    		plotMatches(matches, datasets.imgs{i-1}, datasets.imgs{i}, kpts{i-1}, kpts{i});
    	end
    	hold off;
    	pause(pause_time);
    end
    fig_count = fig_count + 1;
end