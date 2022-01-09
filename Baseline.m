function [nextkeyframe,latdist,fordist] = Baseline(P_3D,Poscam,Rotcam,nframes,xsz,ysz,K)

%% Calculate the number of frames to skip before performing both position
% measurements and new keypoint acceptance

% inputs:
% - 3D positions of the keypoints calculated in bootstrapping or
% prevous iteration as seen by the older keyframe.
% [3,N] (if not, modify lines 65,

% - displacement in distance and angle between cameras
% number of frame changes from keypoint i to i+1
% xsz,ysz the dimensions of the frames in pixels
% img0 the initial keyframe (for dimension purposes)
% K the calibration matrix

% Output:
% - The expected number of frames to skip before finding a keyframe
% - Either: the minimum distances needed between keyframes, either parallel
% or perpendicular to the camera plane
%   Or: the minimum angle formed by (keyframe i, mean keypoint,
% keyframe i+1) and the minimum distance of travel perpendicular to the
% camera plane

% Note: A separate function, CheckIfKeyframe, needs to actually compare
% each new frame against the result of this function. It merely needs to
% check how many frames have passed since the last Baseline.m and trigger
% ValidateKeyframe once the right number of frame changes has been reached.

% Another function, ValidateKeyframe, is necessary to ensure stability. It
% must identify unexpected motion (we moved to little or too much between
% keyframes, or too few keypoints were matched between keyframes i and
% i+1). ValidateKeyframe can instruct the program to instead select an
% earlier or later frame as a keyframe as needed.

%The idea is to create a triangle between the second current keyframe, the
% minimum sidestep length and the minimum forward travel, then copy it by
% symmetry on both sides and front and back. This whole diamond around the
% keyframe would be judged too close to the current frame to be worthy of
% studying in pure translation

% for homogenization, the older current keyframe i (regardless of if i+1 is
% to the left) dictates the origin and axes, if they are necessary

%This function should also verify that sufficiently many keypoints remain
%in view by the time we reach the next keyframe

[c11, c21, c31, c12, c22, c32, c13, c23, c33] = matsplit(Rotcam);

framechange = 1; %1, 0 or -1 ; controls whether to test one frame further or
% stop
i = 1; %contains the frame (after the current latest keyframe) being
% studied

nextkeyframe = 1;

%% Estimate average 3D distance and rotation covered per frame change

%(pos of keyframe i+1 - pos of keyframe i) / (number of frame changes from
%i to i+1 = expected distance traveled per frame

frameposchange = Poscam./nframes;

%these next formulas come from the Robot Dynamics handout
totalrotchangethet = acos((c11+c22+c33-1)/2);
framerotchangethet = totalrotchangethet/nframes;
framerotchangevec = 1/(2*sin(totalrotchangethet))*[c32-c23;c13-c31;c21-c12];

[nx,ny,nz] = matsplit(framerotchangevec);
% c = cos(framerotchangethet);
% s = sin(framerotchangethet);

% framerotchange = [nx^2*(1-c)+c, nx*ny*(1-c)-nz*s, nx*nz*(1-c)+ny*s;...
%     nx*ny*(1-c)+nz*s, ny^2*(1-c)+c, ny*nz*(1-c)-nx*s;...
%     nx*nz*(1-c)-ny*s, ny*nz*(1-c)+nx*s, nz^2*(1-c)+c];

%% Estimate the mean meanline for each new frame

% the meanline of a frame is the 3d line connecting the center of the frame
% to the mean keypoint
% Here, we want the mean between the meanline of the keyframe, and each new
% frame

meankp = sum(P_3D,2)/length(P_3D);
meanline1 = meankp./norm(meankp);

meankp2 = meankp - Poscam;
meanline2 = meankp2./norm(meankp2);

dmeanlineangles = norm(cross(meanline1,meanline2))/nframes;
dmeanlineanglec = dot(meanline1,meanline2)/nframes;
dmeanlineangled = cross(meanline1,meanline2)/norm(cross(meanline1,meanline2));

% meanlinei = [dmeanlineanglec -dmeanlineangles,...  % estimated meanline of the
%     dmeanlineangles dmeanlineanglec]^i *meanline2; % ith next frame
%
% meanmeanline = (meanlinei+meanline2)/2;

%% Calculate the expected sidestep length
%
% % The distance metric used will be the distance between the keyframes in
% % the direction perpendicular to the mean of both frames' meanline
%
% sidestep = i*norm(cross(frameposchange,meanmeanline));
%
% %% Calculate the expected forward travel
%
% % The distance metric used will be the distance between the keyframes in
% % the direction parallel to the mean of both frames' meanline
%
% frontstep = i*norm(dot(frameposchange,meanmeanline));

%% Calculate the exclusion diamond

% calculate the mean Perpendicular distance to a keypoint from the mean of the positions
% of both initial keyframes

% meanposcam = Poscam/2;%%%%%%%%%%%%%%%%%
%
% meandist = norm(dot((Poscam/norm(Poscam)),meankp-meanposcam))/norm(meankp);%%%%%%%%%%%%%%
%meandist = norm(dot(Poscam,meankp-meanposcam));


% Calculate the minimum pure lateral distance needed with the rule of thumb
% keyframe distance = average depth * threshold (10-20%)

% minlat = meandist*.15;
minlat = .05; % adimensional

% Calculate the minimum pure forward distance needed by asking that more
% than, say, 15% of keypoints go out of view between keyframes, assuming
% keyframes are randomly spread in the image. For a camera angle of about
% 90°, this means that the forward motion should be of at least 15% of the
% mean depth of the keypoints according to the second initial keyframe

% minfwd = meandist*.15;
minfwd = .1; %adimensional

latdist = minlat;
fordist = minfwd;

%% Compare these values to the exclusion diamond
%
% % We want to evaluate the equation a*|y|+b*|x|>a*b with x,y the new lateral
% % and frontal positions, and a and b the minimum required for each if it was
% % only a pure lateral or frontal movement
%
% if minlat*frontstep + minfwd*sidestep > minlat*minfwd
%     framechange = 0;
% end
%
% % If false, proceed to next frame
%
% % If true, this is the closest frame that satisfies the distance conditions

%% Make sure that sufficiently many keypoints remain in frame

% Simple check to see, for each successive simulated frame change, more
% than 60% of keypoints remain in frame

% 1) find the rightmost pixel

rmp = xsz/2;
tmp = ysz/2;

% 2) Lecture 2: theta = arctan(Xc/Zc) ; rmp = origin + alpha_u*Xc/Zc ;
%    alpha_u = K(1,1)
%    ==> theta = arctan(rmp/k(1,1))

thetu = atan(rmp/K(1,1));
thetv = atan(tmp/K(2,2));
kpthetacos = dot(P_3D,repmat([0;0;1],1,length(P_3D)),1)...
    ./vecnorm(P_3D);% angles between keypoint-and aimlines
kpthetas = acos(kpthetacos); % assumption that the camera angle < pi
kpthetasin = cross(P_3D,repmat([0;0;1],1,length(P_3D)),1)...
    ./vecnorm(P_3D);
thetmin = min(thetu,thetv);
% thetmin2 = max(kpthetas); %%%%%%%%%%%%%%%

% % 3) Check that this angle is greater than the angles formed by most of the
% % KP after frame change (60%) in abs val
%
% %3D vectors from the new frame to each KP
% links = P_3D-repmat(Poscam+frameposchange*nframes,1,length(P_3D));
%
% %normalized
% directions = links./vecnorm(links);
%
% %compared to the aimline. We compare to the smallest image dimension to
% % simplify comparison. Assume angle<pi
% the aimline of a frame is the 3d line perpendicular to the frame,
% intersecting it in its center
% aimdiff = acos(dot(directions,repmat(aimlinei,length(P_3D)),2));
%
% thetas = arctan(aimdiff(1,:));
%
% thetscore = sum(thetas>thetmin)/length(thetas);
%
%
% %should this check be failed, the loop stops and this function returns the
% %values of the previous loop
% if thetscore>0.4
%
%     framechange = -1;
% else
%     nextkeyframe = i;ii
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Loop%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while framechange==1

    %% Estimate the mean meanline for each new frame

    % Angle Axis notation
    nx2 = dmeanlineangled(1); ny2 = dmeanlineangled(2); nz2 = dmeanlineangled(3);
    c = dmeanlineanglec; s = dmeanlineangles;

    % Rotation Matrix
    Cmeanlines = [nx2^2*(1-c)+c, nx2*ny2*(1-c)-nz2*s, nx2*nz2*(1-c)+ny2*s;...
    nx2*ny2*(1-c)+nz2*s, ny2^2*(1-c)+c, ny2*nz2*(1-c)-nx2*s;...
    nx2*nz2*(1-c)-ny2*s, ny2*nz2*(1-c)+nx2*s, nz2^2*(1-c)+c];


    % estimated meanline of the ith next frame
    norma = @(a) a./norm(a);
    meanlinei =  norma(Cmeanlines^i*meanline2);

    meanmeanline = (meanlinei+meanline2)/2;

    %% Calculate the expected sidestep length

    % The distance metric used will be the distance between the keyframes in
    % the direction perpendicular to the mean of both frames' meanline

%     sidestep = i*norm(cross(frameposchange,meanmeanline));
    sidestep = i*norm(cross(frameposchange,meanmeanline))/norm(meankp);


    %% Calculate the expected forward travel

    % The distance metric used will be the distance between the keyframes in
    % the direction parallel to the mean of both frames' meanline

%     frontstep = i*norm(dot(frameposchange,meanmeanline));
    frontstep = i*norm(dot(frameposchange,meanmeanline))/norm(meankp);

    %% Compare these values to the exclusion diamond

    % We want to evaluate the equation a*|y|+b*|x|>a*b with x,y the new lateral
    % and frontal positions, and a and b the minimum required for each if it was
    % only a pure lateral or frontal movement

    if minlat*frontstep + minfwd*sidestep > minlat*minfwd
        framechange = 0;
    end

    % If false, proceed to next frame

    % If true, this is the closest frame that satisfies the distance conditions

    %% Make sure that sufficiently many keypoints remain in frame

    % 3) Check that this angle is greater than the angles formed by most of the
    % KP after frame change (80%) in abs val

    %3D vectors from the new frame to each KP
    links = P_3D-repmat(Poscam+frameposchange*i,1,length(P_3D));

    %normalized
    directions = links./vecnorm(links);

    %compared to the aimline. We compare to the smallest image dimension to
    % simplify comparison. Assume |angle|<pi
    % the aimline of a frame is the 3d line perpendicular to the frame,
    % intersecting it in its center
    thetai = i*framerotchangethet;
    c = cos(thetai+totalrotchangethet); % total angle = from KPi to i+1 + from
    s = sin(thetai+totalrotchangethet); % i to frame

    framerotchange = [nx^2*(1-c)+c, nx*ny*(1-c)-nz*s, nx*nz*(1-c)+ny*s;...
        nx*ny*(1-c)+nz*s, ny^2*(1-c)+c, ny*nz*(1-c)-nx*s;...
        nx*nz*(1-c)-ny*s, ny*nz*(1-c)+nx*s, nz^2*(1-c)+c];

    aimlinei = framerotchange*[0;0;1];

    cosaimdiff = dot(directions,repmat(aimlinei,[1,length(P_3D)]),1);
    aimdiff = acos(cosaimdiff);

   % thetas = atan(aimdiff(1,:));
   thetas = aimdiff;

    thetscore = sum(thetas>thetmin)/length(thetas);

    %added check to limit keypoint angular travel between frames
    
    newkp = framerotchange.'*directions; % KP seen from the new frame, normalized


    newanglesin = cross(newkp,repmat([0;0;1],1,length(newkp)),1)...
    ./vecnorm(newkp);

    cosangtravel = dot(newanglesin,kpthetasin,1)./vecnorm(newanglesin)...
        ./vecnorm(kpthetasin);
    angtravel = acos(cosangtravel);

    limangletravel = max(kpthetas)/3;%10/360*2*pi; %angle travel cutoff point

    travelscore = sum(angtravel>limangletravel)/length(angtravel);



    %should this check be failed, the loop stops and this function returns the
    %values of the previous loop
    if thetscore>0.3 || travelscore>.4 % more than 30% out of frame or 
                                          % more than 40% travel too far
        framechange = -1;
    else
        nextkeyframe = i;
    end

    i = i+1;

end







%% Return calculated values

% - expected number of keyframes to change till the next operation
% - Exclusion diamond half-diagonals

%% Problems with this implementation

% It fails to take into account the fact that large movements or object
% proximity can hide keypoints or make them unrecognizable
% Suggested fix: add another condition to limit the maximum angle change of
% a known keypoint in the camera frame


end