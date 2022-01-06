function [kpts, desc] = harris(image, min_quality, filter_size)
	% @brief: 	given an image return the Harris keypoints and the coresponding features (descriptors)
	% 
	% @param(image)					: grayscale image
	%
	% @output(kpts)					: num_kpts x 2
	% @output(desc)					: not sure about dim (TODO) but its basically
	%								  the one from Matlab: 
	%								  https://ch.mathworks.com/help/vision/ref/extractfeatures.html
	% 
	% for more info on hyperparam (because I wasnt relly specific here):
	% https://ch.mathworks.com/help/vision/ref/siftpoints.html
	% https://ch.mathworks.com/help/vision/ref/extractfeatures.html#bsxmas0-validPoints
	% https://ch.mathworks.com/help/vision/ref/detectsiftfeatures.html?searchHighlight=sift&s_tid=srchtitle_sift_3

	kp 				= detectHarrisFeatures(image, 'MinQuality', min_quality, 'FilterSize', filter_size);
	[desc, kpts] 	= extractFeatures(image, kp.Location);
	kpts			= round(kpts);
end
