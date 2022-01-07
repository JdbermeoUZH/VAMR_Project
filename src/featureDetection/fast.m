function [kpts, desc] = fast(image, min_quality, min_contrast, max_num_kpts)
	% @brief: 	given an image return the FAST keypoints and the coresponding features (descriptors)
	% 
	% @param(image)					: grayscale image
	%
	% @output(kpts)					: num_kpts x 2
	% @output(desc)					: not sure about dim (TODO) but its basically
	%								  the one from Matlab: 
	%								  https://ch.mathworks.com/help/vision/ref/extractfeatures.html
	% 

	kp 				= detectFASTFeatures(image, 'MinQuality', min_quality, 'MinContrast', min_contrast);
	kp 				= selectStrongest(kp, max_num_kpts);
	[desc, kpts] 	= extractFeatures(image, kp.Location);
	kpts			= round(kpts);