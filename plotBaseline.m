function fig_count = plotBaseline(allimginf,KFidx,transform, abspose,fig_count,abstrans)
%Y,X,Z
    % TODO: Documentation

    numpt = zeros(1,length(allimginf));

for ix = 2:length(allimginf)
    numpt(ix) = length(allimginf(ix).ptcloud);
end
mnumpt = max(numpt);

ptcloud = zeros(mnumpt,3,length(KFidx));

x = abspose(1,4,:); y = abspose(2,4,:); z = abspose(3,4,:);
x = x(:); y = y(:); z = z(:);
xx = abstrans(1,1,:);
yy = abstrans(2,1,:);
zz = abstrans(3,1,:);
figure(fig_count)
scatter3(x,y,z,'ob','MarkerEdgeAlpha',.2)
xlabel('x')
ylabel('y')
zlabel('z')
% set(gca,'DataAspectRatio',[1 .10^-10 10^-10])
hold on
scatter3(x(1),y(1),z(1),'r')
scatter3(xx,yy,zz,'k*')




for index = 1:length(KFidx)
    relp3di = [allimginf(KFidx(index)).ptcloud];
    addit = ones(1,length(relp3di));
    relp3di = transform(:,:,index)*[relp3di;addit];
    relp3di = relp3di(1:3,:);
%     ptcloud(:,:,index) = relp3di;

    pts = relp3di;

    scatter3(pts(1,:),pts(2,:),pts(3,:),'.r','MarkerEdgeAlpha',.2)



end
fig_count = fig_count+1;
hold off
end
