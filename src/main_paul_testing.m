clear all       % lets start over :) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% setup testing environment
% add necesary paths (please do NOT remove this)
addpath('utils/recoverPoseFromFundMatrix/');% some utility functions (mostly long function names)
addpath('utils/plotting/');                 % some utility functions (mostly plotting)
addpath('utils/triangulation/');            % some utility functions (mostly triangulation)
addpath('setup/');		                    % project setup (hyperparams, loading images, ...)
addpath('featureDetection/Harris/');        % Harris
addpath('featureDetection/SIFT/');          % SIFT
addpath('featureMatching/pairwiseFeatureDescriptorComparisson/');     % matching stuff (mostly for SIFT)
addpath('bootstrapping/');                  % yeah bootstrap hurray
% Load stuff
filepaths       = LoadFilePaths();          % get paths to datasets
hyperparameters = LoadHyperParams();        % change ALL params here (no param setting in this file please)
datasets        = LoadProjectImages(hyperparameters, filepaths);  % get our images! 😎
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note: this is a file to test functions! not to be used and loaded
% anywhere else! Please do not change anything here ... Pauls workplace
% only ;) 
%%%%%%%%%%%%%%%%%%%% change the following according to what you want to test %%%%%%%%%%%%
test.bootstrap  = true;
test.sift       = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fig_count       = 1;        % dynamically increase after each new figure
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% lets begin 😎 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TODO: could prpbably make testing functions instaed of implementing it here
%       but I'm lazy :D ... so if you are reading this and have some time 
%       to spare go ahead
%       e.g : 
%       if(test.bootstrap)
%           bootstrapTest(someParameters)
%       end

%% test bootstrapping
if(test.bootstrap)
    % actually call bootstrap (without plotting anything because later on we also don't 
    % want to plot everything ... this would just get messy)
    [R, T, P_3D, matched_keypoints_1, matched_keypoints_2, matches] = bootstrap(datasets, hyperparameters);
    % now plot (because here we are testing so we WANT to plot data)
    figure(fig_count);
    plotBootstrapOutput(P_3D, R, T, ...
        datasets.img0, datasets.img1, ...
        matched_keypoints_1, matched_keypoints_2);
    fig_count = fig_count + 1;
end

%% test sift
if(test.sift)
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
    figure(1);
    for i = img_indices
    	fprintf('\n\nPlot frame %d\n=====================\n', i);
    	% match (skip first frame)
    	if (i > 1)
    		matches = matchFeatures(desc{i-1}, desc{i}, ...
                'MatchThreshold', hyperparameters.match_threshold, ... 
                'MaxRatio', hyperparameters.match_max_ratio, ... 
                'Unique', hyperparameters.match_unique);
    		% plot keypoints that matched
    		plotMatches(matches, datasets.imgs{i-1}, datasets.imgs{i}, kpts{i-1}, kpts{i});
    	end
    	hold off;
    	pause(pause_time);
    end
end