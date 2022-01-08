function [hyperparameters] = LoadHyperParams()

%% Run params
hyperparameters.test					= false;         % Only run the test_range frames in 
hyperparameters.test_range				= 20;          % Number of frames to run the test
%% bootstraping
hyperparameters.bootstrap_frames		= [1 3];
%% hyperparams 8point
hyperparameters.eightPointNumTrials     = 500;          % Default is 500, but acutal NumTrials changes with chose confidence
hyperparameters.eightPointDistanceThreshold = 0.01;     % Default is 0.01
hyperparameters.eightPointConfidence    = 99;           % Default is 99. The higher, the more iterations it requires
%% hyperparams Feature detection
hyperparameters.featDetec_algo_list     = ["Harris", "SIFT", "FAST"];
hyperparameters.featDetec_algo          = "FAST";       % "Harris" or "SIFT" or "FAST"
hyperparameters.featDetec_matchType     = "KLT";        % "KLT" or "Pairwise"
hyperparameters.featDetec_max_num_kpts  = 1000;
%% hyperparameters SIFT
hyperparameters.sift_num_scales 		= 4; 			% 3, 	Scales per octave (num octaves is calculated automatically).
hyperparameters.sift_sigma 				= 1.6;			% 1,    1.6 is recommended in paper (TODO: search for link)
hyperparameters.sift_contrast_threshold	= 0.02;			% 0.04, 0.02 gives more features (and still looks acceptable)
%% hypeparameters Harris
hyperparameters.harris_filer_size       = 5;            % 5,
hyperparameters.harris_min_quality      = 0.01;         % 0.01,
%% hypeparameters FAST
hyperparameters.fast_min_quality        = 0.1;          % 0.1,
hyperparameters.fast_min_contrast       = 0.1;          % 0.2,
%% hypeparameters KLT
hyperparameters.klt_NumPyramidLevels    = 3;            % Default is 3. Number of scales to use. Btw 1 and 4 recommended in matlab doc
hyperparameters.klt_MaxBidirectionalError = 3;        % Default is inf. Error measured in piexels. Btw 0 and 3 recommended in matlab doc
hyperparameters.klt_MaxIterations       = 30;           % Default is 30. Btw 10 and 50 recommended in matlab doc
hyperparameters.klt_BlockSize           = [21, 21];     % Default is [31, 31]. Size of template's box to track. The higher the longer it takes
hyperparameters.klt_withRounding        = true;         % Wether or not to round matched positions to integers or not
%% hyperparameters pairwise matching
hyperparameters.match_threshold			= 100;			% 100,
hyperparameters.match_max_ratio			= 0.7;			% 0.7,
hyperparameters.match_unique			= true;			% true,
hyperparameters.match_withRounding      = false;        % Wether or not to round matched positions to integers or not
%% hypeparameters continous operation
hyperparameters.poseEstimationAlgo      = "8point";     % "8point" or "PnP"
hyperparameters.new_candidate_keypoints_dist_thre = 16; % (distance in pixels)^2 to consider a point as the same point
hyperparameters.bearing_angle_threshold = 5*pi/180;     % Bearing angle at which we can consider safe adding a feature as a landmark
%% hypeparameters reporting
hyperparameters.reporting_window        = 20;           % Last n frames to use in some of the plots
hyperparameters.subtrajectory_lengths   = [3, 5, 7, 10, 20]; % Subtrajectory lengths to use to for RTE
end