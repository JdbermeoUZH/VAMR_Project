function [kpts, desc] = sift(image, num_scales, sigma, contrast_threshold)
	% @brief: 	given an image return the SIFT keypoints and the coresponding features (descriptors)
	% 
	% @param(image)					: grayscale image
	% @param(num_scales)			: number of scales to use 
	% @param(sigma)					: sigam value as learned in lecture
	% @param(contrast_threshold)	: contrast threshold 
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

	kp 				= detectSIFTFeatures(image, Sigma=sigma, NumLayersInOctave=num_scales, ContrastThreshold=contrast_threshold);
	[desc, kpts] 	= extractFeatures(image, kp.Location);
	kpts			= round(kpts);
end
