clear all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% setup testing environment
% add necesary paths (please do NOT remove this)
addpath('utils/recoverPoseFromFundMatrix/');% some utility functions (mostly plotting)
addpath('utils/plotting/');                 % some utility functions (mostly plotting)
addpath('utils/triangulation/');            % some utility functions (mostly plotting)
addpath('setup/');		        % project setup (hyperparams, loading images, ...)
addpath('featureDetection/Harris/');    % Harris
addpath('featureDetection/SIFT/');      % SIFT
addpath('featureMatching/pairwiseFeatureDescriptorComparisson/');     % matching stuff (mostly for SIFT)
addpath('bootstrapping/');      % yeah bootstrap hurray
% Load stuff
filepaths       = LoadFilePaths();                  % get paths to datasets
hyperparameters = LoadHyperParams();                % change ALL params here (no param setting in this file please)
datasets        = LoadProjectImages(hyperparameters, filepaths);              % get our images! 😎
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% test bootstrapping
bootstrap(datasets, hyperparameters);