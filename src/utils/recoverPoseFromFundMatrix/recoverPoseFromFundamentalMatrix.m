function [R,T, P_3D,badpoints] = recoverPoseFromFundamentalMatrix(...
    F, K_1, K_2, points_img_1, points_img_2)
%recoverPoseFromFundamentalMatrix Recover the Pose and Translation from 
%the Fundamental matrix given that we know the intrisic parameters 
%of both cameras.
%   Get essential matrix from fundamental matrix
%   Decompose the Essential matrix to possible rots and translations
%   Disambiguate the actual rotation and translation

% Get essential matrix from fundamental matrix
E = (K_2).' * F * K_1;

% Decompose the Essential matrix to possible rots and translations
[PossibleRotations, PossibleTranslations] = decomposeEssentialMatrix(E);

% Convert 2D points to homogenous coordinates
points_img_1_h = horzcat(points_img_1, ones(length(points_img_1), 1));
points_img_2_h = horzcat(points_img_2, ones(length(points_img_2), 1));

% Disambiguate the actual rotation and translation
[R,T, P_3D,badpoints] = disambiguateRelativePose(...
    PossibleRotations, PossibleTranslations,...
    points_img_1_h.', points_img_2_h.', K_1, K_2);
end

