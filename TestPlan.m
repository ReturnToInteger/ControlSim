clear;
load TestTrack.mat

SensDist=10;
j=1;
theta=0;
Angle=(-pi/6:pi/18:pi/6)';
step_size = 0.5; 
StartPos=VehiclePos(j,:);


dist=vecnorm(ConePos'-StartPos');

CurrentlySeen=ConePos(dist<=SensDist,:)-StartPos;

close all;
figure(1)

plot(CurrentlySeen(:,1),CurrentlySeen(:,2),'o','Color',[0.5 0.5 0.5],'MarkerFaceColor',[0.5 0.5 0.5]);
hold on;
% plot(0,0,'o','Color',[1 0.5 0],'MarkerFaceColor',[1 0.5 0],'MarkerSize',10);

axis([-SensDist SensDist -SensDist SensDist]);
axis equal;

ax=gca;
ax.NextPlot='replaceChildren';

% loops=round(length(VehiclePos(:,1))/100);
loops=1000;

F(loops) = struct('cdata',[],'colormap',[]);


% VhclRelPos=zeros(loops,2);
it=2;
U=[1 0];
[points,~,U]=planPath(CurrentlySeen,Angle,[0 0],0,U,it);
plot(points(:,1),points(:,2))
VhclRelPos=StartPos+points(it,1:2)*step_size;
F=getframe(gcf);
Orient=points(it,3)*step_size;

%0<=conf<=1 for correct results
conf=1;
for j=1:loops

    tic;
    dist=vecnorm((ConePos-VhclRelPos(j,:))');
    SurelySeen=ConePos(dist<=SensDist*conf,:);
    ProbablySeen=ConePos((SensDist*conf)<dist&dist<=SensDist,:);
    ProbablySeen=ProbablySeen(rand(size(ProbablySeen,1),1)<0.3,:);
    CurrentlySeen=[SurelySeen; ProbablySeen]-VhclRelPos(j,:);
    
    hold off;
    clf
    plot(CurrentlySeen(:,1),CurrentlySeen(:,2),'o','Color',[0.5 0.5 0.5],'MarkerFaceColor',[0.5 0.5 0.5])

    [points(:,:,1), cost(1),U]=planPath(CurrentlySeen,Angle,[0 0],Orient,U(end-1,1:2),it);
%     [points(:,:,2), cost(2)]=planPath(CurrentlySeen,Angle,[0 0],points(8,3)-pi/10,8);
%     [points(:,:,3), cost(3)]=planPath(CurrentlySeen,Angle,[0 0],points(8,3)+pi/10,8);

    [~, costi]=min(cost);   
    iter=0.2;
    while (cost(costi)==Inf)
        iter=min(iter+0.01,0.4);
        disp(Orient+" before")
        Orient_temp=Orient+sign(rand-0.5)*(1-rand/2)*pi*iter;
        disp(Orient_temp+" after")

        [points(:,:,1), cost(1),U]=planPath(CurrentlySeen,Angle,[0 0],Orient_temp,U(end-1,1:2),it);
        [~, costi]=min(cost);    
    end
   
    Orient=sum(points(:,3,costi))/sum(1:size(points,1))*step_size;
    VhclRelPos(j+1,:)=VhclRelPos(j,:)+sum(points(:,1:2,costi),1)/sum(1:size(points,1))*step_size;
    hold on;
    fnplt(cscvn(points(1:end,:,costi)'))
    axis equal;
    axis([-SensDist SensDist -SensDist SensDist]);


    drawnow;

    F(j)=getframe(gcf);
    t=toc;
    fdur=0.1;
    if t<fdur
        pause(fdur-t)
    end

end


