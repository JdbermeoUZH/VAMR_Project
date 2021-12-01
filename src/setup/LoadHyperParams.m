%% bootstraping
bootstrap_frames		= [1 3];

%% hyperparameters SIFT
sift_num_scales 		= 4; 			% 3, 	Scales per octave (num octaves is calculated automatically).
sift_sigma 				= 1.6;			% 1,    1.6 is recommended in paper (TODO: search for link)
sift_contrast_threshold	= 0.03;			% 0.04, 0.02 gives more features (and still looks acceptable)

%% hyperparameters matching
match_threshold			= 100;			% 100
match_max_ratio			= 0.7;			% 0.7
match_unique			= true;			% true

%% Run params
test					= true;         % Only run the test_range frames in 
test_range				= 30;           % Number of frames to run the test