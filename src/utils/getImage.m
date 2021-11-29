function image = getImage(image_path, rescale_factor)
	% I don't know if we need this
    image = im2double(imresize(rgb2gray(imread(image_path)),...
        rescale_factor));
end
