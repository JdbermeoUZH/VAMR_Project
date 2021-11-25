function blurred_images = computeBlurredImages(image_pyramid, num_scales, sigma)
	% blurs each image in given image pyramid num_scales often.
	% sigma corresponds to the sigma of the gaussian used for blurring
	% the image.

	% extract number of octaves used in image pyramid build
    	num_octaves 	= numel(image_pyramid);
	% init our return value accordingly
    	blurred_images 	= cell(1, num_octaves);
	% iterate over every octave
    	for oct_idx = 1:num_octaves
		% add 3 to num_scales (best described by fig4 in exercise description)
    	    	imgs_per_oct	= num_scales + 3;
    	    	octave_stack 	= zeros([size(image_pyramid{oct_idx}) imgs_per_oct]);
    	    	for stack_idx 	= 1:imgs_per_oct
			% to start with s=-1
    	    	    	s = stack_idx - 2;
    	    	    	% Hence s in {-1, ..., num_scales + 1} ==> do gaussian
			% smoothing with each "s" value as given by
			% sigma = sigma0 * 2^(s/num_scales)
    	    	    	octave_stack(:, :, stack_idx) = imgaussfilt(image_pyramid{oct_idx}, sigma * 2^(s / num_scales));
       	 	end
       		blurred_images{oct_idx} = octave_stack;
    	end
end
