function [fig_count,allimginf,numtgfis] = baselineTest(datasets, hyperparameters, fig_count)

tic
%This functions currently only calculates local 3D point clouds and the
%relative positions of the keyframes relative to the precedent keyframe

%it runs on the same platform as Bootstrap, so we can launch it from Paul's
%testing base and choose which algorithms to use for each task

%However, I need to use some results which are not returned by
%Bootsrapping, so I call its subfunctions individually. This means that
%updating bootstrap will not also modify this code. I have attempted to
%isolate the pieces of the bootstrapping algorithm I use, so we can just
%ctrl c-v it

%Parking: 46s, 152 frames studied, 69 KF

%Malaga 1st 300frames: 282s, 135 calculated frames, 59 KF







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
        keypoints_2 = nan;
        descriptors_1 = nan;
        descriptors_2 = nan;
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Structures containing useful info about each KF
% Info on each KF is local, in the reference frame of the previous KF
% (except KF1, which is in its own)

    inf1 = struct("frame",hyperparameters.bootstrap_frames(1),"KP",keypoints_1,...
    "des",descriptors_1,"KFmatches",matched_keypoints_1,...
    "framematches",matched_keypoints_1, ...
    "frontstep",0,"sidestep",0,...
    "matchedInliers1",matchedInliers1,...
    "matchedInliers2",matchedInliers2,...
    "FundamentalMatrix",nan,...
    "Rotation",eye(3),...
    "Translation",zeros(3,1),...
    "ptcloud",P_3D, ...
    "KF",1);

    inf2 = struct("frame",hyperparameters.bootstrap_frames(2),"KP",keypoints_2,...
    "des",descriptors_2,"KFmatches",matched_keypoints_2,...
    "framematches",matched_keypoints_2, ...
    "frontstep",0,"sidestep",0,...
    "matchedInliers1",matchedInliers1,...
    "matchedInliers2",matchedInliers2,...
    "FundamentalMatrix",nan,...
    "Rotation",R,...
    "Translation",T,...
    "ptcloud",P_3D, ...
    "KF",1);

bootimginf = [inf1,inf2];


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
% matchednumber = length(matchedInliers1); %%%%%%%%%

while changeframe ~= 0
    [changeframe,imginf] = ValidateKeyframe(hyperparameters,...
                img,tgfi,changeframe,keypoints_2,descriptors_2,minlat,minfwd,imginf,K,img1);

%         img,tgfi,changeframe,keypoints_2,descriptors_2,minlat,minfwd,imginf,matchednumber,K,img1);
    img = datasets.imgs{frame};
end

allimginf = imginf;
alltgfis = zeros(length(datasets.imgs)+hyperparameters.bootstrap_frames(2),1);
alltgfisindx = 1;

%% Iteration

while frame<length(datasets.imgs)
    P_3D = allimginf(end).ptcloud;
    T = allimginf(end).Translation;
    R = allimginf(end).Rotation;
    nframes = frame-keyframe;
    if nframes <=0
        break
    end
    
    [nextkeyframe,minlat,minfwd] = Baseline(P_3D,T,R,nframes,xsz,ysz,K);

    alltgfis(alltgfisindx) = nextkeyframe;
    alltgfisindx = alltgfisindx+1;

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
%     matchednumber = length(allimginf(end).matchedInliers2); %%%%%%%%%
    KFKP = allimginf(end).KP;
    KFdes = allimginf(end).des;
    


    while changeframe ~= 0 && frame<length(datasets.imgs)
        [changeframe,imginf] = ValidateKeyframe(hyperparameters,...
                        img,tgfi,changeframe,KFKP,KFdes,minlat,minfwd,imginf,K,KFimg);

%             img,tgfi,changeframe,KFKP,KFdes,minlat,minfwd,imginf,matchednumber,K,KFimg);
        if frame > length(datasets.imgs)
            frame = length(datasets.imgs);

        end
        img = datasets.imgs{frame};
    end

    if isequal(fieldnames(imginf),fieldnames(allimginf))
        allimginf = [allimginf,imginf];
    end

end

alltgfis = nonzeros(alltgfis);
numtgfis = length(alltgfis);

% Reestablishing correct frame numbering
loopnums = [allimginf(:).frame];
loopnums = loopnums + ones(size(loopnums))*hyperparameters.bootstrap_frames(2);
for index = 1:length(loopnums)
allimginf(index).frame = loopnums(index);
end

allimginf = [bootimginf,allimginf];

% numKF = sum([allimginf(:).KF]);%%%%%%%%%%

%% Absolute pose



rot = [allimginf(:).Rotation];

% rot1 = [allimginf(1).Rotation]; %%%%%%%%%%%%%%
% rot2 = [allimginf(2).Rotation];

rot3dm = reshape(rot,3,3,[]);

trans = [allimginf(:).Translation];

trans3dm = reshape(trans,3,1,[]);

pose = [rot3dm,trans3dm];

additive = zeros(1,4,size(pose,3));
additive(1,end,:) = 1;

pose = [pose;additive];



poseabs = pose;
for i=2:size(pose,3)
    poseabs(:,:,i) = poseabs(:,:,i-1)*poseabs(:,:,i);
end

transform = poseabs;
poseabs = poseabs(1:3,:,:);

abstrans = poseabs(:,4,:);



% scatter3(abstrans(1,1,:),abstrans(2,1,:),abstrans(3,1,:))
% x = abstrans(1,1,:);
x = abstrans(1,1,:);
y = abstrans(2,1,:);
z = abstrans(3,1,:);



x = x(:);
y = y(:);
z = z(:);

figure(fig_count)
subplot(1,2,1)

plot3(x,y,z,'-ob')
xlabel('x')
ylabel('y')
zlabel('z')
% set(gca,'DataAspectRatio',[1 .10^-10 10^-10])
hold on
scatter3(x(1),y(1),z(1),'r')

hold off

fig_count = fig_count+1;

%% 3D Point Cloud


for index = 3:numel(allimginf)
    relp3di = [allimginf(index).ptcloud];
    addit = ones(1,length(relp3di));
    relp3di = transform(:,:,index-1)*[relp3di;addit];
    relp3di = relp3di(1:3,:);
    allimginf(i).ptcloud = relp3di;
end

absp3d = [allimginf(2:end).ptcloud];

subplot(1,2,2)
scatter3(absp3d(1,:),absp3d(2,:),absp3d(3,:))
xlabel('x')
ylabel('y')
zlabel('z')
% plot(absp3d(1,:))
% plot(absp3d(2,:))
% plot(absp3d(3,:))

% sure = vecnorm(absp3d)


%% Intermediary Points

% 
KFidx = find([allimginf(:).KF] == 1); %keyframe indices
knwnf = allimginf(:).frame; %all frames we have records of

for idx = hyperparameters.bootstrap_frames(end)+1:size(datasets.imgs)+hyperparameters.bootstrap_frames(end)

    if sum(knwnf == idx) == 0 %if first encounter with this frame

        %Find best keypoint info to match with
        [~,relkfidx] = min(abs(KFidx-ones(1,length(KFidx))*idx));%index of closest KF in KF list
        relkf = KFidx(relkfidx); %closest KF
        siderelkf = sign(relkf-idx); %whether the kf is before or after the frame
        srelkf = KFidx(relkfidx-siderelkf);
        keypoints_1 = allimginf(relkf).KP;
        descriptors_1 = allimginf(relkf).des;
        if relkf>KFidx(2)
            relkfimg = datasets(idx-hyperparameters.bootstrap_frames(end)).imgs;
        else
            relkfimg = datasets.img1;
        end

%         %Extract inlier KP and des to hopefully lighten calculatory load
%         keypoints_1 = [allimginf(relkf).matchedInliers2;...
%             allimginf(srelkf).matchedInliers2];
%         
%         allrelkfdes = allimginf(relkf).des;
%         [~, pos] = ismember(allimginf(relkf).matchedInliers2,allimginf(relkf).KP,"rows");
        
        
%         relkfindes = allrelkfdes(find(allimginf(relkf).KP == allimginf(relkf).matchedInliers2));
%         allsrelkfdes = datasets(srelkf).des;
%         srelkfindes = allsrelkfdes(find(allimginf(srelkf).KP == allimginf(srelkf).matchedInliers2));       


%         descriptors1 = [relkfindes;srelkfindes];

        fimgidx = idx-hyperparameters.bootstrap_frames(end);
        fimg = datasets.imgs{fimgidx};

        if (featDetec_matchType == "Pairwise")
            % img1
            [keypoints_2, descriptors_2] = featDetect(fimg, hyperparameters);
            % Create vectors with positions of pixels matched in both frames
            [matched_keypoints_1, matched_keypoints_2] = getMatchedPoints(...
                keypoints_2, keypoints_1, descriptors_2, descriptors_1, ...
                hyperparameters);

        elseif (featDetec_matchType == "KLT")
            [matched_keypoints_1, matched_keypoints_2, ~] = ...
                getKLTMatches(relkfimg, keypoints_1, img1, ...
                NumPyramidLevels, MaxBidirectionalError, ...
                MaxIterations, BlockSize);
            keypoints_2 = nan;
            descriptors_1 = nan;
            descriptors_2 = nan;
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

        




    end


    if sum(KFidx == idx) == 0 %if not a KF



    end


end
%
%
%












toc







% Next to do: KLT the intermediary frames
%if done with KP, we can first try to match only with inliers, and then
%take more if it doesn't fly

%Make visualisation better

%Right now, the code is setup so trajectory calculation is done at the very
%end. Should I alter it so that the 














end