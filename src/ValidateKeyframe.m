function [i,frame,changeframe,imginf] = ValidateKeyframe(hyperparameters,i,frame,...
    img,tgfi,changeframe,KFKP,KFdes,minlat,minfwd,imginf,matchednumber,K,KFimg)


%changeframe: 1, -1, or 0. States whether the previous iteration of Val.Key.
%asked to go fwd or back one frame, or no error message

%tgfi target i, the ith frame after the current latest KF, which is predicted
%to be the next KF

%KFKP 2 by x vector, KP in current latest KF

%minlat,minfwd : exclusion diamond distances calculated by Baseline.m

%img_inf is a structure array. Each cell contains the number of the frame
%it has information for, as well as a list of keypoints

global i frame
i = i+1;	%ith frame since the last KF (relative)
frame = frame+1;	%tracks the current frame number (absolute)

if i<tgfi && changeframe~=-1	%if next expected KF not yet reached and no warning
    changeframe = 1;	%we skip this frame
    imginf = struct;
    imgnum = 0;
    return;
end



%
featDetec_matchType         = hyperparameters.featDetec_matchType;
if (featDetec_matchType == "Pairwise")
    [KP, des] = featDetect(img, hyperparameters);
    % Create vectors with positions of pixels matched in both frames
    [KFmatches, framematches] = getMatchedPoints(...
        KP, KFKP, des, KFdes, ...
        hyperparameters);

elseif (featDetec_matchType == "KLT")
    [KFmatches, framematches, ~] = ...
        getKLTMatches(KFimg, KFKP, img, ...
        NumPyramidLevels, MaxBidirectionalError, ...
        MaxIterations, BlockSize);
else
    % output error
    error('The given feature detection method is not valid');
end
%


kpcell = struct("frame",frame,"KP",KP,"KFmatches",KFmatches,...
    "framematches",framematches, ...
    "frontstep",0,"sidestep",0,...
    "matchedInliers1",nan,...
    "matchedInliers2",nan,...
    "FundamentalMatrix",nan,...
    "Rotation",nan,...
    "Translation",nan,...
    "ptcloud",nan);
if isempty(fieldnames(imginf))
    imginf = kpcell;
else
    imginf = [imginf,kpcell];
end


%if not enough KP
if length(framematches) < 0.8*matchednumber	%we prepare to run Val.Key on the previous frame
    changeframe = -1;	%this means anticipating that i and frame  will
    i = i-2;		%be positively incremented
    frame = frame-2;
    if i==1
        changeframe = 0;
    end
    return
end

% %if not enough recognized KP are matched with both KF0 and the current
% %frame
% nummatches = length(intersect(KFmatches,(KPmatches01.'),"rows"));
% if nummatches < 10	%we prepare to run Val.Key on the previous frame
%     changeframe = -1;	%this means anticipating that i and frame  will
%     i = i-2;		%be positively incremented
%     frame = frame-2;
%     if i==1
%         changeframe = 0;
%     end
%     return
% end


% Estimate the pose change with ransac
output = getFundamentalMatrix(KFmatches, framematches, hyperparameters);
F               = output.F;
matchedInliers1 = output.inlierPts1;
matchedInliers2 = output.inlierPts2;


imginf(end).matchedInliers1 = matchedInliers1;
imginf(end).matchedInliers2 = matchedInliers2;
imginf(end).FundamentalMatrix = F;




%if enough KP but next frame is no good
if changeframe==-1
    changeframe = 0;
    return
end


% Get the relative rotaion and translatoin between camera frames. We assume K_1=K_2
[R, T, P_3D] = recoverPoseFromFundamentalMatrix(...
    F, K, K, matchedInliers1, matchedInliers2);

imginf(end).Rotation = R;
imginf(end).Translation = T;
imginf(end).ptcloud = P_3D;

%%%%COPY PASTE HERE THE CODE FOR PARALLEL AND PERPENDICULAR DISTANCES BTW KF AND MEAN KP

%%%%%%%%%%%%%%%%%%%%%%
%if this and next frame have enough KP, but this frame is too close to the KF


% Estimate the mean meanline

% the meanline of a frame is the 3d line connecting the center of the frame
% to the mean keypoint
% Here, we want the mean between the meanline of the keyframe, and the
% frame being tested

meankp = sum(P_3D,2)/length(P_3D);
% meanline1 = meankp./norm(meankp);
%
% meankp2 = meankp - T;
% meanline2 = meankp2./norm(meankp2);
%
% dmeanlineangles = norm(cross(meanline1,meanline2));
% dmeanlineanglec = dot(meanline1,meanline2);
% dmeanlineangled = cross(meanline1,meanline2)/norm(cross(meanline1,meanline2));
%
%
% meanmeanline = (meanline1+meanline2)/norm(meanline1+meanline2);

meanmeanline = (meankp-T./2);

sidestep = norm(cross(T,meanmeanline));

frontstep = norm(dot(T,meanmeanline));

%if outside of exclusion diamond


varside = minlat*frontstep + minfwd*sidestep;
coefside = minlat*minfwd;

% Plot for Debug
%plotPoseEstimation(P_3D, R, T, matchedInliers1, matchedInliers2, KFimg, img, 1);
%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if varside < coefside %INSIDE EXCLUSION DIAMOND
    changeframe = 1;
    return;
end

imginf(end).frontstep = frontstep;
imginf(end).sidestep = sidestep;

changeframe = 0;



end