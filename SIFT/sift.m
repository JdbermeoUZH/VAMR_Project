function [descriptors, kpt_locations] = computeDescriptors( ...
		image, num_octaves, num_scales, sigma, contrast_threshold, rotation_inv);

    	image_pyramid 		= computeImagePyramid(image, num_octaves);
    	blurred_images 		= computeBlurredImages(image_pyramid, num_scales, sigma);
    	DoGs 			= computeDoGs(blurred_images);
    	tmp_kpt_locations	= extractKeypoints(DoGs, contrast_threshold);
    	[descriptors, kpt_locations] =...
    	    computeDescriptors(blurred_images, tmp_kpt_locations, rotation_inv);
