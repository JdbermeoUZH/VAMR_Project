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
addpath('featureDetection/');               % general feature detection

addpath('featureMatching/pairwiseFeatureDescriptorComparisson/');     % matching stuff (mostly for SIFT)
addpath('featureMatching/KLT');             % matching stuff (mostly for SIFT)

addpath('ransac/');

addpath('continuousOperation/');

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
test.bootstrap  = false;
test.contOp     = false;
test.baseline   = true;

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
    [fig_count, matched_keypoints_1, matched_keypoints_2, P_3D] = ... 
            bootstrapTest(datasets, hyperparameters, fig_count);
    
end

%% test Continous Operation
if(test.contOp || test.all)
    fprintf('\n\n Test Continous Operation \n=====================\n');
    fig_count = continousPoseEstimationTest( ...
        datasets, hyperparameters, fig_count);
end



%% test Baseline Pipeline

if test.baseline
    fprintf('\n\n Test Baseline Pipeline \n=====================\n');
    hyperparameters.featDetec_algo = "SIFT";
    fig_count = baselineTest(datasets, hyperparameters, fig_count);
end


fprintf('\n=============================================================\n');
fprintf('====================== Testing done ☑️ ======================');
fprintf('\n=============================================================\n');



