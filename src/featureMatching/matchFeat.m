function [matchedPts1, matchedPts2, validity] = matchFeat(img0, kpts0, img1, hyperparameters, desc0)

    if (hyperparameters.featDetec_matchType == "Pairwise")
        if ~exist('desc0', 'var')
	        [desc0, ~] = extractFeatures(img0, kpts0);
        end
        % img1
        [kpts1, desc1] = featDetect(img1, hyperparameters);
        % Create vectors with positions of pixels matched in both frames
        [matchedPts1, matchedPts2, matches] = getPairwiseMatches(...
            kpts1, kpts0, desc1, desc0, ...
            hyperparameters);
            
        validity = ones(size(matchedPts1,1),1); % hacky way to make all the points valid :D

    elseif (hyperparameters.featDetec_matchType == "KLT")
        NumPyramidLevels            = hyperparameters.klt_NumPyramidLevels;           
        MaxBidirectionalError       = hyperparameters.klt_MaxBidirectionalError;        
        MaxIterations               = hyperparameters.klt_MaxIterations;           
        BlockSize                   = hyperparameters.klt_BlockSize; 
        withRounding                = hyperparameters.klt_withRounding;
        [matchedPts1, matchedPts2, validity] = ...
            getKLTMatches(img0, kpts0, img1, ...
                          NumPyramidLevels, MaxBidirectionalError, ...
                          MaxIterations, BlockSize, withRounding);
    else
        % output error
        error('The given feature matching method is not valid');
    end
end