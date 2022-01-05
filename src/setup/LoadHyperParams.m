function [hyperparameters] = LoadHyperParams()

%% Run params
hyperparameters.test					= true;         % Only run the test_range frames in 
hyperparameters.test_range				= 100;           % Number of frames to run the test
%% bootstraping
hyperparameters.bootstrap_frames		= [1 3];
%% hyperparams 8point
hyperparameters.eightPointNumTrials     = 30000;        % Default is 500, but acutal NumTrials changes with chose confidence
hyperparameters.eightPointDistanceThreshold = 0.01;     % Default is 0.01
hyperparameters.eightPointConfidence    = 99;           % Default is 99. The higher, the more iterations it requires
%% hyperparams Feature detection
hyperparameters.featDetec_algo          = "SIFT";       % "Harris" or "SIFT"
hyperparameters.featDetec_matchType     = "KLT";   % "KLT" or "Pairwise"
%% hyperparameters SIFT
hyperparameters.sift_num_scales 		= 4; 			% 3, 	Scales per octave (num octaves is calculated automatically).
hyperparameters.sift_sigma 				= 1.6;			% 1,    1.6 is recommended in paper (TODO: search for link)
hyperparameters.sift_contrast_threshold	= 0.02;			% 0.04, 0.02 gives more features (and still looks acceptable)
%% hypeparameters Harris (TODO: make naming consistent e.g. start with "harris_")
hyperparameters.corner_patch_size       = 9;            %9,
hyperparameters.harris_kappa            = 0.12;         %0.08,  between 0.04 and 0.15 smaller seems better on parking lot. lesson 5 slide 47
hyperparameters.num_keypoints           = 150;          %200,
hyperparameters.nonmaximum_supression_radius = 10;      %8,     bigger ==> descriptors are further apart
hyperparameters.descriptor_radius       = 9;            %9,
hyperparameters.match_lambda            = 6;            %4,     Bigger lambda ==> less selective in matching descriptors
%% hypeparameters KLT
hyperparameters.klt_NumPyramidLevels    = 3;            % Default is 3. Number of scales to use. Btw 1 and 4 recommended in matlab doc
hyperparameters.klt_MaxBidirectionalError = inf;        % Default is inf. Error measured in piexels. Btw 0 and 3 recommended in matlab doc
hyperparameters.klt_MaxIterations       = 30;           % Default is 30. Btw 10 and 50 recommended in matlab doc
hyperparameters.klt_BlockSize           = [31, 31];     % Default is [31, 31]. Size of template's box to track. The higher the longer it takes
%% hyperparameters pairwise matching
hyperparameters.match_threshold			= 100;			% 100,
hyperparameters.match_max_ratio			= 0.7;			% 0.7,
hyperparameters.match_unique			= true;			% true,
%% hypeparameters continous operation
hyperparameters.poseEstimationAlgo      = "8point";     % "8point" or "P3P"
hyperparameters.new_candidate_keypoints_dist_thresh = 16; % (distance in pixels)^2 to consider a point as the same point
hyperparameters.bearing_angle_threshold = 5*pi/180;     % Bearing angle at which we can consider safe adding a feature as a landmark
%% hypeparameters reporting
hyperparameters.reporting_window        = 20;           % Last n frames to use in some of the plots
hyperparameters.subtrajectory_lengths   = [3, 7, 15, 23, 30, 50, 100]; % Subtrajectory lengths to use to for RTE
end