function relscales = scalematching(allimginf,hyperparameters,K)

% Input:
%   struct containing all known info on the Keyframes
%   hyperparameters
%   datasets.K

% Output: vector containing the relative scales of each new KF pair's
% triangulation compared to the first one


% Extract the real KF from allimginf

KFidx = find([allimginf(:).KF] == 1);
numKF = length(KFidx);

%% Extract all KF KP and des, inliers and 3D Points for consecutive frames

numKP = zeros(length(allimginf),1);
numin = numKP;
numdesx = numKP;
numdesy = numKP;
for ix = 1:length(allimginf)
    numKP(ix) = length(allimginf(ix).KP);
    numin(ix) = length(allimginf(ix).matchedInliers1);
    numdesx(ix) = size(allimginf(ix).KP,1);
    numdesy(ix) = size(allimginf(ix).KP,2);
end
mnumKP = max(numKP);
mnumin = max(numin);
mnumdesx = max(numdesx);
mnumdesy = max(numdesy);




KP = zeros(mnumKP,2,numKF);
des = zeros(mnumdesx,121,numKF);
inliersn = zeros(mnumin,2,numKF);
p3d2 = zeros(mnumin,3,numKF);
i2 = 1;

for ix = KFidx

    KP(:,:,i2) = padarray([allimginf(ix).KP],[mnumKP,2] - size([allimginf(ix).KP]),'post');
    des(:,:,i2) = padarray([allimginf(ix).des],[mnumdesx,121] - size([allimginf(ix).des]),'post');
    inliersn(:,:,i2) = padarray([allimginf(ix).matchedInliers1],[mnumin,2] - size([allimginf(ix).matchedInliers1]),'post');
    p3d2(:,:,i2) = padarray([allimginf(ix).ptcloud],[3,mnumin] - size([allimginf(ix).ptcloud]),'post').';
    i2 = i2+1;

end


relscales = zeros(numKF-2,1);

for pairidx = 1:numKF-2


    %% Match ntn KP and generate 1st inliers

    keypoints_1 = KP(:,:,pairidx);
    keypoints_1 = keypoints_1(any(keypoints_1,2),:);
    keypoints_2 = KP(:,:,pairidx+2);
    keypoints_2 = keypoints_2(any(keypoints_2,2),:);
    descriptors_1 = des(:,:,pairidx);
    descriptors_1 = descriptors_1(any(descriptors_1,2),:);
    descriptors_2 = des(:,:,pairidx+2);
    descriptors_2 = descriptors_2(any(descriptors_2,2),:);

    [matched_keypoints_2, matched_keypoints_1] = getMatchedPoints(...
        keypoints_2, keypoints_1, descriptors_2, descriptors_1, ...
        hyperparameters);

    % Estimate the pose change with ransac
    output = getFundamentalMatrix(matched_keypoints_1, matched_keypoints_2, hyperparameters);
    F               = output.F;
    matchedInliers1 = output.inlierPts1;
    matchedInliers2 = output.inlierPts2;

    %% Add matches with n inliers to ntn inliers




    % Get the relative rotaion and translatoin between camera frames. We assume K_1=K_2
    [rot, ~, P_3D,badpoints] = recoverPoseFromFundamentalMatrix(...
        F, K, K, matchedInliers1, matchedInliers2);

    matchedInliers1(badpoints,:) = [];
    matchedInliers2(badpoints,:) = [];

    inliersntn = matchedInliers1;
    p3dntn = P_3D.';

%     plot3(p3d2(:,1),p3d2(:,2),p3d2(:,3),'r')
%     hold on
% plot3(p3dntn(:,1),p3dntn(:,2),p3dntn(:,3),'b')

try

    [poi,idxn,idxntn] = intersect(inliersn(:,:,pairidx+1),inliersntn,'rows'); % points of interest

    p3dntn = p3dntn(idxntn,:);

    p3dn = p3d2(idxn,:,pairidx+1);
    
    
    %     [pointn,~,idxntn] = intersect(poi,matchedInliers1,'rows'); % points of interest
%     matchedInliers1 = union(matchedInliers1,poi,'rows');

%     scatter(inliers2(:,1,pairidx),inliers2(:,2,pairidx),'r')
%     hold on
%     scatter(matched_keypoints_1(:,1),matched_keypoints_1(:,2),'b')
    % intersect(keypoints_1,matched_keypoints_1,'rows')
    %

% 
%     poi2 = matched_keypoints_2(idx,:);
%     matchedInliers2 = union(matchedInliers2,poi2,'rows');
% 
%     
%     [poin,~,idxn] = intersect(poi,inliersn(:,:,pairidx+1),'rows'); % points of interest
% 




    %% Compare neighbors with next-to-neighbors (ntn)


    %     cmnpts = intersect(inliers2(:,:,pairidx),inliers3,'rows'); %inliers in both n and ntn
%     pts1 = p3d2(idxn,:,pairidx); %3d pos of common KP in frame n
%     pts2 = p3dntn(:,idxntn); %3d pos of common KP in frame ntn
%     dist1 = norm(diff(pts1)); %distances btw consecutive 3d pts in frame n
%     dist2 = norm(diff(pts2)); %distances btw consecutive 3d pts in frame ntn
%     sclntn = mean(dist1./dist2); %scaling to convert distances from ntn to n
%     scl2 = sclntn/(1-sclntn);

%     distn = mean(vecnorm(diff(p3dn),2,2)); %distances btw consecutive 3d pts in frame n
%     distntn = mean(vecnorm(diff(p3dntn),2,2)); %distances btw consecutive 3d pts in frame ntn
%     sclntn = mean(distn./distntn); %scaling to convert distances from ntn to n
%     scl2 = sclntn/(1-sclntn);

[x0n,~] = find(p3dn < 0);
[x0ntn,~] = find(p3dntn < 0);
x0 = [x0n;x0ntn];
p3dn(x0,:) = [];
p3dntn(x0,:) = [];
p3dn = p3dn(any(p3dn,2),:);
p3dntn = p3dntn(any(p3dn,2),:);
p3dn = p3dn(any(p3dntn,2),:);
p3dntn = p3dntn(any(p3dntn,2),:);



    meanptn = mean(p3dn,1);

    ptdistn = vecnorm(p3dn-repmat(meanptn,[length(p3dn) 1]),2,2);
    distn = std(ptdistn);



    meanptntn = mean(p3dntn,1);
    ptdistntn = vecnorm(p3dntn-repmat(meanptntn,[length(p3dntn) 1]),2,2);
    distntn = std(ptdistntn);


%     ptdistn(ptdistn > distn*10) = [];
%     ptdistn(ptdistntn > distn*10) = [];    
%     
%     
% 
%     ptdistntn(ptdistntn > distntn*10) = [];
%     ptdistntn(ptdistn > distn*10) = [];
% 
%     distn = std(ptdistn);
%     distntn = std(ptdistntn);


%     scl2 = distn/distntn -1;

    %angles:

    calpha = meanptn'*meanptntn; %angle cos btw n and ntn representations of mean KP
    salpha= cross(meanptn,meanptntn);

    alpha = atan(calpha/salpha);

    scl2 = sqrt(1+(distn/distntn)^2-2*distn/distntn*abs(calpha));




%     figure
% scatter3(p3dn(:,1),p3dn(:,2),p3dn(:,3),'r')
%     hold on
% scatter3(p3dntn(:,1),p3dntn(:,2),p3dntn(:,3),'b')
% 
% 
% for i = 1:length(p3dntn)
%     
%     x = [p3dn(i,1),p3dntn(i,1)];
%     y = [p3dn(i,2),p3dntn(i,2)];
%     z = [p3dn(i,3),p3dntn(i,3)];
% 
%     plot3(x,y,z,'k')
% 
% end

% We have few points, and high instability ==> lots of errors
% I could spend another 30 hours calculatingbetter points to go off of, or
% maybe a better way to calculate scale altogether.
% Oooooor, I could just replace all really erroneous results with
% an approximation based on the number of frames changed and the hypothesis
% that framerate and speed are locally constant
if (isnan(scl2)|| scl2<0) %&& pairidx>2
    scl2 = (allimginf(KFidx(pairidx+2)).frame-...
        allimginf(KFidx(pairidx+1)).frame)/...
    (allimginf(KFidx(pairidx+1)).frame-...
    allimginf(KFidx(pairidx)).frame);
end
catch
    scl2 = (allimginf(KFidx(pairidx+2)).frame-...
        allimginf(KFidx(pairidx+1)).frame)/...
    (allimginf(KFidx(pairidx+1)).frame-...
    allimginf(KFidx(pairidx)).frame);
end
    relscales(pairidx) = scl2;


end
    
scales = relscales;
for sclidx = 2:length(scales)
    scales(sclidx) = scales(sclidx)*scales(sclidx-1);
end


end