function [hyperparameters] = LoadHyperParams()

%% Run params
hyperparameters.test					= true;         % Only run the test_range frames in 
hyperparameters.test_range				= 30;           % Number of frames to run the test

%% bootstraping
hyperparameters.bootstrap_frames		= [1 3];

%% hyperparams Feature detection
hyperparameters.featDetec_algo          = "SIFT";       % "Harris" or "SIFT"

%% hyperparams Ransac
hyperparameters.ransac_algo             = "8point";     %       which algo to use

%% hyperparameters SIFT
hyperparameters.sift_num_scales 		= 4; 			% 3, 	Scales per octave (num octaves is calculated automatically).
hyperparameters.sift_sigma 				= 1.6;			% 1,    1.6 is recommended in paper (TODO: search for link)
hyperparameters.sift_contrast_threshold	= 0.03;			% 0.04, 0.02 gives more features (and still looks acceptable)

%% hypeparameters Harris (TODO: make naming consistent e.g. start with "harris_")
hyperparameters.corner_patch_size       = 9;            %9,
hyperparameters.harris_kappa            = 0.12;         %0.08,  between 0.04 and 0.15 smaller seems better on parking lot. lesson 5 slide 47
hyperparameters.num_keypoints           = 150;          %200,
hyperparameters.nonmaximum_supression_radius = 10;      %8,     bigger ==> descriptors are further apart
hyperparameters.descriptor_radius       = 9;            %9,
hyperparameters.match_lambda            = 6;            %4,     Bigger lambda ==> less selective in matching descriptors

%% hyperparameters matching
hyperparameters.match_threshold			= 100;			% 100,
hyperparameters.match_max_ratio			= 0.7;			% 0.7,
hyperparameters.match_unique			= true;			% true,

end