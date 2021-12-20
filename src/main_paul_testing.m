
clear all       % lets start over :) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% setup testing environment
% add necesary paths (please do NOT remove this)
addpath('utils/recoverPoseFromFundMatrix/');% some utility functions (mostly long function names)
addpath('utils/plotting/');                 % some utility functions (mostly plotting)
addpath('utils/triangulation/');            % some utility functions (mostly triangulation)
addpath('setup/');		            % project setup (hyperparams, loading images, ...)
addpath('featureDetection/Harris/');        % Harris
addpath('featureDetection/SIFT/');          % SIFT
addpath('featureMatching/pairwiseFeatureDescriptorComparisson/');     % matching stuff (mostly for SIFT)
addpath('continuousOperation/');
addpath('featureMatching/KLT');             % matching stuff (mostly for SIFT)
addpath('bootstrapping/');                  % yeah bootstrap hurray
addpath('tests/');                          % super cool testing
% Load stuff
filepaths       = LoadFilePaths();          % get paths to datasets
hyperparameters = LoadHyperParams();        % change ALL params here (no param setting in this file please)
datasets        = LoadProjectImages(hyperparameters, filepaths);  % get our images! 😎
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note: this is a file to test functions! not to be used and loaded
% anywhere else!
%%%%%%%%%%%%%%%%%%%% change the following according to what you want to test %%%%%%%%%%%%
test.harris     = false;
test.sift       = false;
test.bootstrap  = true;
test.contOp     = false;

test.all        = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fig_count       = 1;        % dynamically increase after each new figure
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% lets begin 😎 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('\n=============================================================\n');
fprintf('======================= Start Testing =======================');
fprintf('\n=============================================================\n');

%% test harris
if(test.harris || test.all)
    fprintf('\n\n Test Harris \n=====================\n');
    hyperparameters.featDetec_algo = "Harris";
    fig_count = harrisTest(datasets, hyperparameters, fig_count);
end

%% test sift
if(test.sift || test.all)
    fprintf('\n\n Test SIFT \n=====================\n');
    hyperparameters.featDetec_algo = "SIFT";
    fig_count = siftTest(datasets, hyperparameters, fig_count);
end

%% test bootstrapping
if(test.bootstrap || test.all)
    fprintf('\n\n Test Bootstraping \n=====================\n');
    % with SIFT + Pairwise test:
    hyperparameters.featDetec_algo = "SIFT";
    hyperparameters.featDetec_matchType = "Pairwise";
    [fig_count, matched_keypoints_1_sift, matched_keypoints_2_sift] = ... 
            bootstrapTest(datasets, hyperparameters, fig_count);
    
    % with SIFT + KLT test:
    hyperparameters.featDetec_algo = "SIFT";
    hyperparameters.featDetec_matchType = "KLT";
    [fig_count, matched_keypoints_1_sift, matched_keypoints_2_sift] = ... 
            bootstrapTest(datasets, hyperparameters, fig_count);
    
    % with Harris + Pairwise test:
    hyperparameters.featDetec_algo = "Harris";
    hyperparameters.featDetec_matchType = "Pairwise";
    [fig_count, matched_keypoints_1_harris, matched_keypoints_2_harris] = ... 
            bootstrapTest(datasets, hyperparameters, fig_count);
    
    % with Harris + KLT test:
    hyperparameters.featDetec_algo = "Harris";
    hyperparameters.featDetec_matchType = "KLT";
    [fig_count, matched_keypoints_1_harris, matched_keypoints_2_harris] = ... 
            bootstrapTest(datasets, hyperparameters, fig_count);
end

%% test Continous Operation
if(test.contOp || test.all)
    fprintf('\n\n Test Continous Operation \n=====================\n');
    fig_count = continousPoseEstimationTest( ...
        datasets, hyperparameters, fig_count);
end

fprintf('\n=============================================================\n');
fprintf('====================== Testing done ☑️ ======================');
fprintf('\n=============================================================\n');
