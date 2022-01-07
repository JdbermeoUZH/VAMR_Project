function [kpts, desc] = harris(image, min_quality, filter_size, max_num_kpts)
	% @brief: 	given an image return the Harris keypoints and the coresponding features (descriptors)
	% 
	% @param(image)					: grayscale image
	%
	% @output(kpts)					: num_kpts x 2
	% @output(desc)					: not sure about dim (TODO) but its basically
	%								  the one from Matlab: 
	%								  https://ch.mathworks.com/help/vision/ref/extractfeatures.html
	% 

	kp 				= detectHarrisFeatures(image, 'MinQuality', min_quality, 'FilterSize', filter_size);
	kp 				= selectStrongest(kp, max_num_kpts);
	[desc, kpts] 	= extractFeatures(image, kp.Location);
	kpts			= round(kpts);
end
