function [ aligned_X ] = umeyama( X, Y, estimateScale, plotResult )
%UMEYAMA Corresponding point set registration with Umeyama method.
%
% [aligned_X] = umeyama(X, Y)   returns aligned trajectory after finding the 
% the rotation matrix R and translation % vector t and scale s that 
% approximate Y = sR * X + t using least-squares estimation. X
% and Y are in format [3xn] and point X(:,i) corresponds to point Y(:,i)
% for all i.
%
% [aligned_X] = umeyama(X, Y, true)  returns the same result but in addition a
% figure is created plotting the registration result and the average
% registration error.
%
% Author: Christoph Graumann, 2015
%   Chair for Computer Aided Medical Procedures and Augmented Reality
%   Technische Universitaet Muenchen (Munich, Germany) and 
%   Johns Hopkins University (Baltimore, MD, USA)
%
% Accessed Decemeber 29th from https://github.com/cgraumann/umeyama-matlab/blob/master/umeyama.m 

assert(size(X,1)==size(Y,1) && size(X,2)==size(Y,2),'Dimensions of matrices must match!');
assert(size(X,1)==3, 'The points must be given in format [3xn]');

%% Demean
m = size(X,1);
n = size(X,2);
mean_X = mean(X,2);
mean_Y = mean(Y,2);
X_demean = X - repmat(mean_X,1,size(X,2));
Y_demean = Y - repmat(mean_Y,1,size(Y,2));

%% SVD
sigma = 1/n*Y_demean*X_demean';
[U, D, V] = svd(sigma);

%% Define S
S = eye(m);
if det(sigma) < 0 || (rank(sigma) == m-1 && det(U)*det(V) < 0)
    S(m,m) = -1;
end

%% Bootstrap

% Find the scale
if (estimateScale)
    St=S.';
    traceProduct = D(:).'*St(:);
    scale_x = 1/mean(X_demean(1,:).^2);    % 1/sigma_x
    if scale_x >= 10 scale_x = 1/traceProduct; end
    scale_y = 1/mean(X_demean(2,:).^2);    % 1/sigma_y
    if scale_y >= 10 scale_y = 1/traceProduct; end
    scale_z = 1/mean(X_demean(3,:).^2);    % 1/sigma_z
    if scale_z >= 10 scale_z = 1/traceProduct; end

    scale = traceProduct* [scale_x 0 0;
                           0 scale_y 0;
                           0 0 scale_z];   
else
    scale = 1;
end

R = scale * U*S*V';
t = mean_Y - R*mean_X;

% With the found R and T, align the trajectory
aligned_X = [R t; 0 0 0 1] * [X;ones(1,size(X,2))]; 
aligned_X = aligned_X(1:3, :);

%% Plotting
if nargin>2 && plotResult
    figure('name','Result of Umeyama registration');
    scatter3(X(1,:),X(2,:),X(3,:),'g*');
    hold on;
    scatter3(Y(1,:),Y(2,:),Y(3,:),'bo');
    X_prime = [R t; 0 0 0 1] * [X;ones(1,size(X,2))];
    scatter3(aligned_X(1,:),aligned_X(2,:),aligned_X(3,:),'r*');
    axis equal tight;
    legend('Source points','Destination points','Transformation result');
    MEAN_REGISTRATION_ERROR = norm(mean(abs(Y - X_prime(1:3,:)),2))
end

end
