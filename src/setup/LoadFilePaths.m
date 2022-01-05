function [filepaths] = LoadFilePaths()

filepaths.parking_path			= '../datasets/parking';	% change to your dataset location
filepaths.kitti_path  			= '../datasets/kitti';      % change to your dataset location
filepaths.malaga_path			= '../datasets/malaga';     % change to your dataset location

%% Which dataset to use
filepaths.ds                    = 2;                        % 0: KITTI, 1: Malaga, 2: parking

end