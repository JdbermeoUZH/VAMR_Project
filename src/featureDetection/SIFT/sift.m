function [kpts, desc] = sift(image, num_scales, sigma, contrast_threshold)
	% @brief: 	given an image return the SIFT keypoints and the coresponding features (descriptors)
	% 
	% @param(image)					: grayscale image
	% @param(num_scales)			: number of scales to use 
	% @param(sigma)					: sigam value as learned in lecture
	% @param(contrast_threshold)	: contrast threshold 
	% 
	% for more info on hyperparam (because I wasnt relly specific here):
	% https://ch.mathworks.com/help/vision/ref/siftpoints.html
	% https://ch.mathworks.com/help/vision/ref/extractfeatures.html#bsxmas0-validPoints
	% https://ch.mathworks.com/help/vision/ref/detectsiftfeatures.html?searchHighlight=sift&s_tid=srchtitle_sift_3

	kp 				= detectSIFTFeatures(image, Sigma=sigma, NumLayersInOctave=num_scales, ContrastThreshold=contrast_threshold);
	[desc, kpts] 	= extractFeatures(image, kp.Location);
	% flip coordinates because we want [u v] and not [v u]
	kpts			= [kpts(:,2), kpts(:,1)];
end
