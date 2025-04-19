% clear
% deltamax=pi/4;
% delta=-deltamax:deltamax/6:deltamax;
% cellSize=2;
% 
% 
% 
% %planPath(Obstacle map, Possible steering angles, StartPosition(y,x), Start
% %Heading(negative is upwards), Goal Position(y,x)
% 
% Map=imread('map8.png');
% 
% b=planPath(Map, delta, [640,280], 0, [590,220]);
clear
clc

h=0.1;

v=15;
steering=[0 0];
x0=[-2 -3];
th0=0;
%map

a=pi;
A=200;
t=0:0.01*pi:A*pi*1.5-0.0001;
fx=A*sin(t/A);
fy=2*(A-A*cos(t/A));
M=[fx' fy'];

c=car(x0,th0,v,steering,h,2,M);


%steer(c,pi/4);

% d=car(10,[0 0],h,2);
% steer(d,-pi/4);
% 
% e=car(10,[0 0],h,2);
% steer(e,0);

it=2000;




F=zeros(it,3);
C=zeros(it,3);
R=zeros(it,3);
f=zeros(it,1);
F(1,:)=c.Front(1:3);
C(1,:)=c.Centre(1:3);
R(1,:)=c.Rear(1:3);
f(1,1)=c.f;

fdur=h;
tic;

for i=2:it
    dd=-pi/2:pi/8:pi/2;
    m=zeros(length(dd),1);
    for j=1:length(dd)
        a=steer(copy(c),dd(j));
        updateState(a);
%         for l=1:length(dd)
%             b=steer(copy(a),dd(j));
%             updateState(b);
%             for n=1:length(dd)
%                 d=steer(copy(b),dd(j));
%                 updateState(d);
%                 m(j,l,n)=d.f;
% 
%             end
%         end
        m(j)=a.f;
    end

    
%     [f(i,1), k]=min(min(min(m,[],3),[],2));
    [f(i,1), k]=min(m);
    steer(c,dd(k));
    try
        c.updateState();
    catch
        break;
    end
%     if abs(c.Centre(3)-pi/2)<0.1 && c.f>20
%         disp(i+". "+c.f+" "+c.Centre(1));
%     end
    
    F(i,:)=c.Front(1:3);
    C(i,:)=c.Centre(1:3);
    R(i,:)=c.Rear(1:3);
    f(i,1)=c.f;


end
toc
disp("Simulation time: "+ it*h +" seconds.")
%%
close all
figure
plot(C(:,1),C(:,2))
%plot(R(:,1),R(:,2))

hold on

%plot(F(:,1),F(:,2))

% plot(R(:,1),R(:,2))
plot([F(1,1),C(1,1),R(1,1)],[F(1,2),C(1,2),R(1,2)],'k')
nol=floor(10/h);
w=0.5;
 plot([F(1:nol:end,1)-w*cos(F(1:nol:end,3)) F(1:nol:end,1)+w*cos(F(1:nol:end,3))]',[F(1:nol:end,2)-w*sin(F(1:nol:end,3)) F(1:nol:end,2)+w*sin(F(1:nol:end,3))]','r','LineWidth',4)
 plot([R(1:nol:end,1)-w*cos(R(1:nol:end,3)) R(1:nol:end,1)+w*cos(R(1:nol:end,3))]',[R(1:nol:end,2)-w*sin(R(1:nol:end,3)) R(1:nol:end,2)+w*sin(R(1:nol:end,3))]','g','LineWidth',4)

%plot([C(1:nol:end,1)-w*cos(C(1:nol:end,3)) C(1:nol:end,1)+w*cos(C(1:nol:end,3))]',[C(1:nol:end,2)-w*sin(C(1:nol:end,3)) C(1:nol:end,2)+w*sin(C(1:nol:end,3))]','r','LineWidth',4)


plot([F(1:nol:end,1),C(1:nol:end,1),R(1:nol:end,1)]',[F(1:nol:end,2),C(1:nol:end,2),R(1:nol:end,2)]','k','LineWidth',1)
plot(fx,fy,'--')
axis equal
%axis([-min(C(1:end,1)) -max(C(1:end,1)) (min([C(1:end,2); -1])-2) max([C(1:end,2);1])+2])
hold off
figure
plot((1:1:it),f)
