%% bootstraping
bootstrap_frames		= [1 3];

%% hyperparameters SIFT
sift_num_scales 		= 4; 			% 3, 	Scales per octave (num octaves is calculated automatically).
sift_sigma 				= 1.6;			% 1,    1.6 is recommended in paper (TODO: search for link)
sift_contrast_threshold	= 0.03;			% 0.04, 0.02 gives more features (and still looks acceptable)
%% hypeparameters Harris
corner_patch_size       = 9; %9
harris_kappa            = 0.12; %0.08           between 0.04 and 0.15 smaller seems better on parking lot. lesson 5 slide 47
num_keypoints           = 150; %200
nonmaximum_supression_radius = 10; %8    bigger ==> descriptors are further apart
descriptor_radius       = 9; %9
match_lambda            = 6; %4        Bigger lambda ==> less selective in matching descriptors
%% hyperparameters matching
match_threshold			= 100;			% 100
match_max_ratio			= 0.7;			% 0.7
match_unique			= true;			% true

%% Run params
test					= true;         % Only run the test_range frames in 
test_range				= 30;           % Number of frames to run the test