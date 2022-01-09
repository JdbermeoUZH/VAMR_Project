clear all       % lets start over :) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% setup testing environment
% add necesary paths (please do NOT remove this)
addpath('utils/recoverPoseFromFundMatrix/');% some utility functions (mostly long function names)
addpath('utils/plotting/');                 % some utility functions (mostly plotting)
addpath('utils/triangulation/');            % some utility functions (mostly triangulation)
addpath('utils/trajectoryAlignment/');      % some utility functions
addpath('utils/movie/');

addpath('setup/');		                    % project setup (hyperparams, loading images, ...)

addpath('featureDetection/');               % general feature detection

addpath('featureMatching/');                % matching stuff

addpath('8point/');

addpath('continuousOperation/');

addpath('bootstrapping/');                  % yeah bootstrap hurray

addpath('tests/');                          % super cool testing
%%%%%%%%%%%%%%%%%%%% change the following according to what you want to test %%%%%%%%%%%%
test.harris     = false;
test.sift       = false;
test.bootstrap  = false;
test.contOp     = false;
test.EVERYTHING = true;

test.all        = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load stuff
filepaths       = LoadFilePaths();          % get paths to datasets
hyperparameters = LoadHyperParams();        % change ALL params here (no param setting in this file please)
if (~test.EVERYTHING)
    datasets        = LoadProjectImages(hyperparameters, filepaths);  % get our images! 😎
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note: this is a file to test functions! not to be used and loaded
% anywhere else!
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
    [fig_count]  = continousPoseEstimationTest( ...
        datasets, hyperparameters, fig_count);
end

if(test.EVERYTHING)
    fprintf('\n\n Test EVERYTHING \n=====================\n');
    parfor i = 3:8    
        parTesting(i, hyperparameters, filepaths);
    end
end
    

fprintf('\n=============================================================\n');
fprintf('====================== Testing done ☑️ ======================');
fprintf('\n=============================================================\n');

function parTesting(i, hyperparameters, filepaths)
    filepaths.ds    = idivide(i, int32(3), 'floor');
    if (i<3)
        j = i+1;
    elseif (i<6)
        j = i-2;
    else
        j = i-5;
    end
    hyperparameters.featDetec_algo = hyperparameters.featDetec_algo_list(j);
    datasets        = LoadProjectImages(hyperparameters, filepaths);
    fig_count       = i*2+1;
    [fig_count]     = continousPoseEstimationTest( ...
                        datasets, hyperparameters, fig_count);
end