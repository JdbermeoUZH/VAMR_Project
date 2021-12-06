function [fig_count] = harrisTest(datasets, hyperparameters, fig_count)
    % Ideas to make more recognizable:
    % line recognition: if some features are aligned for two consecutive frames, expect them to be for the rest
    % only match points if the distance is not too big
    % predict feature movement based on position in image (y position for example in parking lot
    
    % Harris
    harris_scores = harris(datasets.img0, ... 
                        hyperparameters.corner_patch_size, ...
                        hyperparameters.harris_kappa);
    assert(min(size(harris_scores) == size(harris_scores)));

    %Display
    figure('Color', 'w');
    subplot(fig_count, 2, 1);
    imshow(datasets.img0);
    subplot(fig_count, 2, 2);
    imshow(datasets.img0);

    fig_count = fig_count + 1;

    subplot(fig_count, 2, 4);
    imagesc(harris_scores);
    title('Harris Scores');
    daspect([1 1 1]);

    axis off;
    fig_count = fig_count + 1;


    %% Part 2 - Select keypoints

    keypoints = selectKeypoints(harris_scores, ... 
                    hyperparameters.num_keypoints, ... 
                    hyperparameters.nonmaximum_supression_radius);
    figure(fig_count);
    imshow(datasets.img0);
    hold on;
    plot(keypoints(2, :), keypoints(1, :), 'rx', 'Linewidth', 2);
    fig_count = fig_count + 1;

    %% Part 3 - Describe keypoints and show 16 strongest keypoint descriptors

    descriptors = describeKeypoints(datasets.img0, ... 
                    keypoints, hyperparameters.descriptor_radius);
    figure(fig_count);
    for i = 1:16
        subplot(fig_count, 4, i);
        patch_size = 2 * hyperparameters.descriptor_radius + 1;
        imagesc(uint8(reshape(descriptors(:,i), [patch_size patch_size])));
        axis equal;
        axis off;
    end
    fig_count = fig_count + 1;

    %% Part 4 - Match descriptors between first two images
    % img1 = imread('../data/000001.png');
    harris_scores_2 = harris(datasets.img1, ... 
                        hyperparameters.corner_patch_size, ... 
                        hyperparameters.harris_kappa);
    keypoints_2 = selectKeypoints(harris_scores_2, ... 
                        hyperparameters.num_keypoints, ...
                        hyperparameters.nonmaximum_supression_radius);
    descriptors_2 = describeKeypoints(datasets.img1, ... 
                        keypoints_2, ... 
                        hyperparameters.descriptor_radius);

    matches = matchDescriptors(descriptors_2, descriptors, ... 
                        hyperparameters.match_lambda);

    % TODO: PLOTTING MATCHES BROKEN FOR HARRIS
    %figure(fig_count);
    %imshow(datasets.img1);
    %hold on;
    %plot(keypoints_2(2, :), keypoints_2(1, :), 'rx', 'Linewidth', 2);
    %plotMatches(matches, datasets.img0, datasets.img1, keypoints_2, keypoints);
    %fig_count = fig_count + 1;

    %% Part 5 - Match descriptors between all images
    figure(fig_count);
    img_indices = 1:size(datasets.imgs, 2);
    clear prev_desc
    for i = img_indices
        %img = imread(sprintf('../data/%06d.png',i));
        img = datasets.imgs{i};

        scores = harris(img, ...
                    hyperparameters.corner_patch_size, ...
                    hyperparameters.harris_kappa);
        kp = selectKeypoints(scores, ...
                    hyperparameters.num_keypoints, ...
                    hyperparameters.nonmaximum_supression_radius);
        plot(kp(2, :), kp(1, :), 'rx', 'Linewidth', 2);

        desc = describeKeypoints(img, kp, hyperparameters.descriptor_radius);
        if (exist('prev_desc', 'var'))
            matches = matchDescriptors(desc, prev_desc, hyperparameters.match_lambda);
            % TODO: PLOTTING MATCHES BROKEN FOR HARRIS
            %plotMatches(matches, datasets.imgs{i-1}, datasets.imgs{i}, kp, prev_kp);
        end

        prev_kp = kp;
        prev_desc = desc;
        hold off;
        pause(0.01);
    end
    fig_count = fig_count + 1;
end