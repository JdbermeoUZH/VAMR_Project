function kpt_locations = extractKeypoints(DoGs, contrast_threshold)
	% extract number of octaves used
    	num_octaves 	= numel(DoGs);
	% init return value size
    	kpt_locations 	= cell(1, num_octaves);
    	for oct_idx = 1:num_octaves
    	   	DoG = DoGs{oct_idx};
    	   	% This is some neat trick to avoid for loops. <- (I'll take the TA's
		% word for it here)
    	   	DoG_max = imdilate(DoG, true(3, 3, 3));
		% get keypoints
    	   	is_kpt = (DoG == DoG_max) & (DoG >= contrast_threshold);
    	   	% We do not consider the extrema at the boundaries of the DoGs.
    	   	is_kpt(:, :, 1) = false;
    	   	is_kpt(:, :, end) = false;
		% save locations of keypoints
    	   	[x, y, s] = ind2sub(size(is_kpt), find(is_kpt));
    	   	kpt_locations{oct_idx} = horzcat(x, y, s);
    	end
end
