%%%KITTI DATASET
hyperparameters.klt_MaxBidirectionalError = 0.8;
hyperparameters.klt_NumPyramidLevels = 4; 
hyperparameters.klt_BlockSize = [21 21];  
hyperparameters.klt_MaxIterations = 40;

hyperparameters.eightPointNumTrials = 32000;
hyperparameters.eightPointConfidence = 90;
hyperparameters.eightPointDistanceThreshold = 0.01;

hyperparameters.bearing_angle_thre = 3/180*pi;
hyperparameters.new_candidate_keypoints_dist_thre = 10;

hyperparameters.MinQuality = 1e-4;
hyperparameters.ROI = [];
hyperparameters.FilterSize = 9;

%%%MALAGA DATASET
hyperparameters.klt_MaxBidirectionalError = 0.8;
hyperparameters.klt_NumPyramidLevels = 4; 
hyperparameters.klt_BlockSize = [21 21];  
hyperparameters.klt_MaxIterations = 40;

hyperparameters.eightPointNumTrials = 50000;
hyperparameters.eightPointConfidence = 90;
hyperparameters.eightPointDistanceThreshold = 0.003;

hyperparameters.bearing_angle_thre = 0.5/180*pi;
hyperparameters.new_candidate_keypoints_dist_thre = 15;

hyperparameters.MinQuality = 1e-4;
hyperparameters.ROI = [];
hyperparameters.FilterSize = 9;

%%%PARKING DATASET
hyperparameters.klt_MaxBidirectionalError = 0.8;
hyperparameters.klt_NumPyramidLevels = 6; 
hyperparameters.klt_BlockSize = [21 21];  
hyperparameters.klt_MaxIterations = 40;

hyperparameters.eightPointNumTrials = 32000;
hyperparameters.eightPointConfidence = 97;
hyperparameters.eightPointDistanceThreshold = 0.01;

hyperparameters.bearing_angle_thre = 1.5/180*pi;
hyperparameters.new_candidate_keypoints_dist_thre = 15;

hyperparameters.MinQuality = 1e-4;
hyperparameters.ROI = [];
hyperparameters.FilterSize = 9;

