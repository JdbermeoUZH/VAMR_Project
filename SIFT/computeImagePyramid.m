function image_pyramid = computeImagePyramid(image, num_octaves)
	% Take one image and build a image pyramide (image at different
	% scales).

	% init size of the pyramid
	image_pyramid = cell(1, num_octaves);
	% init first element of our pyramid
    	image_pyramid{1} = image;
    	for idx = 2:num_octaves
		% fill with resized images
		image_pyramid{idx} = imresize(image_pyramid{idx - 1}, 0.5);
    	end
end
