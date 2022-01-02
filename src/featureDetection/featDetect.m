function [kpts, desc] = featDetect(image, hyperparameters)
    % check which algo to use
    if (hyperparameters.featDetec_algo == "SIFT")
        [kpts, desc] = sift(image, ...
                            hyperparameters.sift_num_scales, ...
                            hyperparameters.sift_sigma, ...
                            hyperparameters.sift_contrast_threshold);
    elseif (hyperparameters.featDetec_algo == "Harris")
        [kpts, desc] = getHarrisFeatures(image, ...
                            hyperparameters.corner_patch_size,...
                            hyperparameters.harris_kappa,...
                            hyperparameters.num_keypoints,...
                            hyperparameters.nonmaximum_supression_radius,...
                            hyperparameters.descriptor_radius);
    end
end