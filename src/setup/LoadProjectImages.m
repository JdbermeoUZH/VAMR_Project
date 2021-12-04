function [dataset] = LoadProjectImage(hyperparameters, filepaths)

% filepaths
ds          = filepaths.ds;
kitti_path  = filepaths.kitti_path;
parking_path= filepaths.parking_path;
malaga_path = filepaths.malaga_path;
% hyperparams
test        = hyperparameters.test;
test_range  = hyperparameters.test_range;
bootstrap_frames    = hyperparameters.bootstrap_frames;


if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/05.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    dataset.K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    dataset.K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    dataset.K = load([parking_path '/K.txt']);

    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
if ds == 0
    dataset.img0 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    dataset.img1 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    dataset.img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    dataset.img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    dataset.img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    dataset.img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

%% Continuous operation 
%(added test range for shorter runtime)
if test
	range = (bootstrap_frames(2)+1):test_range;
else
	range = (bootstrap_frames(2)+1):last_frame;
end

%
dataset.imgs    = cell(1,2);
j = 1;
%
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
%         image = imread([parking_path ...
%             sprintf('/images/img_%05d.png',i)]);
    else
        assert(false);
    end
    % Makes sure that plots refresh.
    pause(0.01);
    dataset.imgs{j} = image;  % I dont get why we use "albums" and not just the {} form
    j = j+1;
end
end
