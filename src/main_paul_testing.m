clear all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% setup testing environment
% add necesary paths (please do NOT remove this)
addpath('utils/recoverPoseFromFundMatrix/');% some utility functions (mostly plotting)
addpath('utils/plotting/');                 % some utility functions (mostly plotting)
addpath('utils/triangulation/');            % some utility functions (mostly plotting)
addpath('setup/');		                    % project setup (hyperparams, loading images, ...)
addpath('featureDetection/Harris/');        % Harris
addpath('featureDetection/SIFT/');          % SIFT
addpath('featureMatching/pairwiseFeatureDescriptorComparisson/');     % matching stuff (mostly for SIFT)
addpath('bootstrapping/');                  % yeah bootstrap hurray
% Load stuff
filepaths       = LoadFilePaths();                  % get paths to datasets
hyperparameters = LoadHyperParams();                % change ALL params here (no param setting in this file please)
datasets        = LoadProjectImages(hyperparameters, filepaths);              % get our images! 😎
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note: this is a file to test functions! not to be used and loaded
% anywhere else! Please do not change anything here ... Pauls workplace
% only ;) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
test.bootstrap  = true;
test.sift       = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% lets begin 😎 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% test bootstrapping
if(test.bootstrap)
    [R, T, P_3D, matched_keypoints_1, matched_keypoints_2] = bootstrap(datasets, hyperparameters);
    plotBootstrapOutput(P_3D, R, T, ...
        datasets.img0, datasets.img1, ...
        matched_keypoints_1, matched_keypoints_2);
end

%% test sift
if(test.sift)
    sift()
end