%% bootstraping
bootstrap_frames		= [1 3];

%% hyperparameters SIFT
sift_num_scales 		= 3; 			% 3, 	Scales per octave (num octaves is calculated automatically).
sift_sigma 				= 1.6;			% 1,
sift_contrast_threshold	= 0.04;			% 0.04,

%% hyperparameters matching
match_threshold			= 100;			% 100
match_max_ratio			= 0.7;			% 0.7
match_unique			= true;			% true