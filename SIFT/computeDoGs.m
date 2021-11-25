function DoGs = computeDoGs(blurred_images)
	% take blurred image pyramid and return DoG pyramid

	% extract number of octaves used
    	num_octaves	= numel(blurred_images);
	% init size of our return value
    	DoGs 		= cell(1, num_octaves);
	% iterate over every size of our image
    	for oct_idx = 1:num_octaves
		% init DoG for this octave (will be 1 less than blurred
		% images available since we take the difference of those)
    	    	DoG =  zeros(size(blurred_images{oct_idx})-[0 0 1]);
    	    	num_dogs_per_octave = size(DoG, 3);
		% Calculate the differnece between all neighboring scales
    	    	for dog_idx = 1:num_dogs_per_octave
    	    	   	DoG(:, :, dog_idx) = abs(...
    	    	   	    blurred_images{oct_idx}(:, :, dog_idx + 1) - ...
    	    	   	    blurred_images{oct_idx}(:, :, dog_idx));
    	    	end
		% fill return object
    	    	DoGs{oct_idx} = DoG;
    	end
end
