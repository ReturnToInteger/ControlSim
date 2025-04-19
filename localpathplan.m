clear;
TreeDepth=9;
Angle=(-pi/8:pi/8:pi/8)';
l=0.5;
N=3;

F=zeros((N^(TreeDepth+1)-1)/(N-1)-1,3);

F(1:3,:)=[cos(Angle)*l sin(Angle)*l Angle];

prev=0;

    
for i=1:TreeDepth-1
    for j=1:3^i
        prev=prev+1;
        cur=3*prev+1:3*prev+3;
        F(cur,:)=[F(prev,1)+l*cos(F(prev,3)+Angle) F(prev,2)+l*sin(F(prev,3)+Angle) F(prev,3)+Angle];
    end
    
end

x=zeros(TreeDepth+1,N^TreeDepth);
y=zeros(TreeDepth+1,N^TreeDepth);
dg=zeros(TreeDepth+1,N^TreeDepth);

    
for j=1:TreeDepth
%     sizeofTree=(N^(j+1)-1)/(N-1);
    dg(j+1,:)=repmat((sizeofTree(j-1,N):sizeofTree(j,N)-1)',N^(TreeDepth-j),1);
    dg(j+1,:)=sort(dg(j+1,:));
    
    x(j+1,:)=F(dg(j+1,:),1);
    y(j+1,:)=F(dg(j+1,:),2);    
    
%     theta(j+1,:)=repmat(F(sizeofTree:sizeofTree+N^j-1,3),N^(TreeDepth-j),1);
end

figure(1);
%     'PaperUnits','inches',...
%     'PaperPosition',[0 0 1920 1080]/150,...
%     'PaperPositionMode','manual',...
%     'Visible','off');
plot(x(:,1:end),y(:,1:end));
axis equal;
grid on;
% print('-r150','-dtiff','tree.tiff');