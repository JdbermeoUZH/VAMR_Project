function [output] = getFundamentalMatrix(matchedPts1, matchedPts2, hyperparameters)
    % TODO: Documentation

    % first get inliers from matched points (using ransac)
    [ output.F , output.inliers, output.status ] = estimateFundamentalMatrix(matchedPts1, matchedPts2, ...
                    'Method','RANSAC',...
                    'NumTrials', hyperparameters.eightPointNumTrials, ...
                    'DistanceThreshold', hyperparameters.eightPointDistanceThreshold,...
                    'Confidence', hyperparameters.eightPointConfidence);

    % extract inliers
    output.inlierPts1 = matchedPts1(output.inliers,:);
    output.inlierPts2 = matchedPts2(output.inliers,:);
    % now use 8 point algo
   % output.F = estimateFundamentalMatrix(output.inlierPts1,output.inlierPts2,'Method','Norm8Point');
    
end
    