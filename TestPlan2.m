clear;
load TestTrack.mat

SensDist=15;
j=1;
theta=0;
Angle=(-pi/10:pi/10:pi/10)';

dist=vecnorm(ConePos'-VehiclePos(j,:)');

CurrentlySeen=ConePos(dist<=SensDist,:)-VehiclePos(j,:);

close all;
figure(1)

plot(CurrentlySeen(:,1),CurrentlySeen(:,2),'o','Color',[0.5 0.5 0.5],'MarkerFaceColor',[0.5 0.5 0.5]);
hold on;
% plot(0,0,'o','Color',[1 0.5 0],'MarkerFaceColor',[1 0.5 0],'MarkerSize',10);

axis([-SensDist SensDist -SensDist SensDist]);
axis equal;

ax=gca;
ax.NextPlot='replaceChildren';

loops=length(VehiclePos(:,1));

F(loops) = struct('cdata',[],'colormap',[]);


% VhclRelPos=zeros(loops,2);

points=planPath(CurrentlySeen,Angle,[0 0],VhclAbsHead(1),8);
plot(points(:,1),points(:,2))
F=getframe(gcf);
costi=1;

for j=1:1000:loops
    
    tic;
    dist=vecnorm((ConePos-VehiclePos(j,:))');
    CurrentlySeen=ConePos(dist<=SensDist,:)-VehiclePos(j,:);
    
    hold off;
    plot(CurrentlySeen(:,1),CurrentlySeen(:,2),'o','Color',[0.5 0.5 0.5],'MarkerFaceColor',[0.5 0.5 0.5])

    [points(:,:,1), cost(1)]=planPath(CurrentlySeen,Angle,[0 0],points(5,3,costi),8);
    [points(:,:,2), cost(2)]=planPath(CurrentlySeen,Angle,[0 0],points(8,3,costi)-pi/10,8);
    [points(:,:,3), cost(3)]=planPath(CurrentlySeen,Angle,[0 0],points(8,3,costi)+pi/10,8);
%     
    [~, costi]=min(cost);
    
    hold on;
    plot(points(:,1,costi),points(:,2,costi))
    axis([-SensDist SensDist -SensDist SensDist]);
    axis equal;
    drawnow;

%     F(j)=getframe(gcf);
    t=toc;
    if t<0.25
        pause(0.25-t)
    end

end


