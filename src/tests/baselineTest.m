function [fig_count] = harrisTest(datasets, hyperparameters, fig_count)


%This functions currently only calculates local 3D point clouds and the
%relative positions of the keyframes relative to the precedent keyframe

%it runs on the same platform as Bootstrap, so we can launch it from Paul's
%testing base and choose which algorithms to use for each task

%However, I need to use some results which are not returned by
%Bootsrapping, so I call its subfunctions individually. This means that
%updating bootstrap will not also modify this code. I have attempted to
%isolate the pieces of the bootstrapping algorithm I use, so we can just
%ctrl c-v it







%% Calling Bootstrap Externally
% [R, T, P_3D, matchedInliers1, matchedInliers2] = bootstrap(datasets, hyperparameters);
% 
% nframes = diff(hyperparameters.bootstrap_frames);
% [ysz,xsz] = size(datasets.img0);
% K = datasets.K;
% 
% [nextkeyframe,minlat,minfwd] = Baseline(P_3D,T,R,nframes,xsz,ysz,K);
% 
% global i frame
% i = nextkeyframe;
% frame = i;
% tgfi = nextkeyframe;
% changeframe = 1;
% KF = datasets.img1;
% [KFKP, KFdes] = featDetect(KF, hyperparameters);
% [KF0, KF0des] = featDetect(datasets.img0, hyperparameters);
% imginf = struct;
% img = datasets.imgs{frame};
% matchednumber = length(matchedInliers1);
% KFimg = datasets.img1;
% 
% while changeframe ~= 0
%     [i,frame,changeframe,imginf] = ValidateKeyframe(hyperparameters,i,frame,img,tgfi,changeframe,KFKP,KFdes,minlat,minfwd,imginf,matchednumber,K,KFimg);
%     img = datasets.imgs{frame};
% end
% 
% allimginf = imginf;
% 
% 
% test = 0;

%% Interrupted Bootstrap

    % @brief:   depending on the algorithms set in the given hyperparameters we will 
    %           calculate the rotation, translation and a 3D point cloud of matched keypoints
    %           between two given images (from datasets). As a bounus we also return all
    %           matched keypoints (for plotting ... not necessarily needed)
    %
    % @param(datasets)          :   datasets variables loaded from LoadProjectImages
    % @param(hyperparameters)   :   hyperparameters loaded from LoadHyperParams
    %
    % @return(R)                :   rotation matrix between two cameras
    % @return(T)                :   translation vector between two cameras
    % @return(P_3D)             :   3D point cloud of matched points 
    % @return(matched_keypoints_1): matched (inlier) points in image from camera 1 (with outliers removed)
    % @return(matched_keypoints_2): matched (inlier) points in image from camera 2 (with outliers removed)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Simplify long var names
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    img0    = datasets.img0;
    img1    = datasets.img1;
    K       = datasets.K;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % could also just use hyperparameters.the_thing_to_use in every function 
    featDetec_matchType         = hyperparameters.featDetec_matchType;
    
    % KLT Matching Stuff
    NumPyramidLevels            = hyperparameters.klt_NumPyramidLevels;           
    MaxBidirectionalError       = hyperparameters.klt_MaxBidirectionalError;        
    MaxIterations               = hyperparameters.klt_MaxIterations;           
    BlockSize                   = hyperparameters.klt_BlockSize; 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Lets Go %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Find the keypoint correspondences between the two images
    % --> extract keypoints and features (descriptors)
    % img0
    [keypoints_1, descriptors_1] = featDetect(img0, hyperparameters);
    if (featDetec_matchType == "Pairwise")
        % img1
        [keypoints_2, descriptors_2] = featDetect(img1, hyperparameters);
        % Create vectors with positions of pixels matched in both frames
        [matched_keypoints_1, matched_keypoints_2] = getMatchedPoints(...
            keypoints_2, keypoints_1, descriptors_2, descriptors_1, ...
            hyperparameters);

    elseif (featDetec_matchType == "KLT")
        [matched_keypoints_1, matched_keypoints_2, ~] = ...
            getKLTMatches(img0, keypoints_1, img1, ...
                          NumPyramidLevels, MaxBidirectionalError, ...
                          MaxIterations, BlockSize);
    else
        % output error
        error('The given feature detection method is not valid');
    end

    % Estimate the pose change with ransac
    output = getFundamentalMatrix(matched_keypoints_1, matched_keypoints_2, hyperparameters);
    F               = output.F;
    matchedInliers1 = output.inlierPts1;
    matchedInliers2 = output.inlierPts2;

    % Get the relative rotaion and translatoin between camera frames. We assume K_1=K_2
    [R, T, P_3D] = recoverPoseFromFundamentalMatrix(...
        F, K, K, matchedInliers1, matchedInliers2);



%
nframes = diff(hyperparameters.bootstrap_frames);
[ysz,xsz] = size(datasets.img0);
K = datasets.K;

[nextkeyframe,minlat,minfwd] = Baseline(P_3D,T,R,nframes,xsz,ysz,K);

alltgfis = nextkeyframe;

global i frame
i = nextkeyframe;
frame = i;
keyframe = 0; %earlier keyframe
tgfi = nextkeyframe;
changeframe = 1;
imginf = struct;
img = datasets.imgs{frame};
matchednumber = length(matchedInliers1);

while changeframe ~= 0
    [changeframe,imginf] = ValidateKeyframe(hyperparameters,...
        img,tgfi,changeframe,keypoints_2,descriptors_2,minlat,minfwd,imginf,matchednumber,K,img1);
    img = datasets.imgs{frame};
end

allimginf = imginf;

while frame<length(datasets.imgs)
    P_3D = allimginf(end).ptcloud;
    T = allimginf(end).Translation;
    R = allimginf(end).Rotation;
    nframes = frame-keyframe;
    if nframes <=0
        break
    end
    
    [nextkeyframe,minlat,minfwd] = Baseline(P_3D,T,R,nframes,xsz,ysz,K);

    alltgfis = [alltgfis,nextkeyframe];

    keyframe = frame;
    i = nextkeyframe;
    frame = frame + i;
    if frame > length(datasets.imgs)
        frame = length(datasets.imgs);
    end
    tgfi = nextkeyframe;
    changeframe = 1;
    imginf = struct;
    KFimg = img;
    img = datasets.imgs{frame};
    matchednumber = length(allimginf(end).matchedInliers2);
    KFKP = allimginf(end).KP;
    KFdes = allimginf(end).des;


    while changeframe ~= 0 && frame<length(datasets.imgs)
        [changeframe,imginf] = ValidateKeyframe(hyperparameters,...
            img,tgfi,changeframe,KFKP,KFdes,minlat,minfwd,imginf,matchednumber,K,KFimg);
        if frame > length(datasets.imgs)
            frame = length(datasets.imgs);

        end
        img = datasets.imgs{frame};
    end

    allimginf = [allimginf,imginf];


end





test = 0;



















end