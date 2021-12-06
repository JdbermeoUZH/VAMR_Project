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
addpath('tests/');                          % super cool testing
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
fprintf('\n=============================================================\n');
fprintf('======================= Start Testing =======================');
fprintf('\n=============================================================\n');

%% test bootstrapping
if(test.bootstrap)
    fprintf('\n\n Test Bootstraping \n=====================\n');
    bootstrapTest(datasets, hyperparameters, fig_count);
end

%% test sift
if(test.sift)
    fprintf('\n\n Test SIFT \n=====================\n');
    siftTest(datasets, hyperparameters, fig_count);
end