function [points, cost,Uv]=planPath(Cones,delta,PosStart,HeadingStart,Uv,iterations)
% planPath(Cones,delta,PosStart,HeadingStart,iterations)
% delta a kormányszög tömbben, a többi szerintem egyértelmû
showAnimation=false;

% /*/*/*/*/*/*/*/*/*/ heurisztika start /*/*/*/*/*/*/*/*/*/

% diszkretizálás, késõbb hasznos NextPoint.g számolásánál
% MapWidth = cont2disc(size(Cones,2));
% MapHeight = cont2disc(size(Cones,1));
% Heuristics = zeros(MapHeight,MapWidth);
% 
% for i = 1:MapWidth
%    for j = 1:MapHeight
%         Heuristics(j,i) = (norm([GoalPos(1)/cellSize-j,GoalPos(2)/cellSize-i])*cellSize-60); %euklideszi távolság lesz a heurisztika, ha nem volt akadály
%         if Heuristics(j,i)<0
%             Heuristics(j,i)=0;
%         end
% %	end
%    end
% end

% /*/*/*/*/*/*/*/*/*/* heurisztika end */*/*/*/*/*/*/*/*/*/

discXposStart=cont2disc(PosStart);
g=0;
%h mindig a diszkrét pozíció függvénye
f=g+Heuristics(PosStart,Cones,HeadingStart,HeadingStart);

%writing start position values to a structure that is going to be used
StartPoint.discpos=discXposStart;
StartPoint.heading=HeadingStart;
StartPoint.x=PosStart;
StartPoint.uv=Uv(1);
StartPoint.ud=Uv(2);
StartPoint.g=g;
StartPoint.f=f;
StartPoint.np=[];

%creating an x-element structure
sizeofOpen=50;
Openset=struct('x',cell(1,sizeofOpen),'uv',cell(1,sizeofOpen),'ud',cell(1,sizeofOpen),'heading',cell(1,sizeofOpen),'discpos',cell(1,sizeofOpen),'g',...
     cell(1,sizeofOpen),'f',cell(1,sizeofOpen),'np',cell(1,sizeofOpen));
 
 
 %filling it up to make it comparable to other arrays
 for i=1:sizeofOpen
     Openset(i).f=Inf;
     Openset(i).discpos=[Inf,Inf];
 end
 Openset(1)=StartPoint;


sizeofClosed=150;
Closedset=struct('x',cell(1,sizeofClosed),'uv',cell(1,sizeofClosed),'ud',cell(1,sizeofClosed),'heading',cell(1,sizeofClosed),'discpos',cell(1,sizeofClosed),'g',...
    cell(1,sizeofClosed),'f',cell(1,sizeofClosed),'np',cell(1,sizeofClosed));
 for i=1:sizeofClosed
     Closedset(i).f=Inf;
     Closedset(i).discpos=[Inf,Inf];
 end
 
n=0;
m=0;

% plot(Cones)
hold on;
% plot(GoalPos(2),GoalPos(1),'ro','LineWidth', 14, 'MarkerSize', 14)
plot(PosStart(1),PosStart(2),'o','Color',[1 0.5 0],'MarkerFaceColor',[1 0.5 0],'MarkerSize',10);
if showAnimation==true
    drawnow;
end

%algorithm starts
while ~isempty(Openset)
    
    %finding min f value
    mini=Inf;
    
    k=9999;
    OpensetSize=size(Openset');
    for i=1:OpensetSize(1)
        if Openset(i).f<mini
            mini=Openset(i).f;
            k=i;
        end
    end
    if k==9999
        break
    end

    CurrentPoint=Openset(k);
    plot(CurrentPoint.x(1),CurrentPoint.x(2),'g.','LineWidth', 4, 'MarkerSize', 10)
    if showAnimation==true
        drawnow;
    end
    Openset(k)=[];
    n=n+1;
    Closedset(n)=CurrentPoint;
    
    if CurrentPoint.g==iterations+1
        %megtaláltuk a célt, ezért visszaadjuk az utat
        f=zeros(n,3);

%         save
        
        f(1,1:2)=CurrentPoint.x;
        f(1,3)=CurrentPoint.heading;
        cost=CurrentPoint.f;
        Uv=[CurrentPoint.uv, CurrentPoint.ud];
        dummy=CurrentPoint;
        q=1;
       
        
        while not(isempty(dummy.np))
            for i=1:n
                if Closedset(i).discpos==dummy.np
                    dummy=Closedset(i);
                    q=q+1;
                    f(q,1:2)=Closedset(i).x;
                    f(q,3)=Closedset(i).heading;
                    Uv(q,1:2)=[Closedset(i).uv, Closedset(i).ud];
                    break
                end
            end
        end
        points=f(1:iterations+2,:);

        title('Hybrid A* Animation');
%         disp('Hurray! You found a path!');
        
        return;
    else
        %ha nem találtuk meg, megy az algoritmus tovább
        
        [Openset, Closedset, n,m]=updateNeighbours(Cones,delta,Openset,Closedset,CurrentPoint,n,m,HeadingStart);
    end
    
end
        disp('I finished it completely');
        points(1:iterations+2,1:2)= repmat(PosStart,iterations+2,1);
        cost=Inf;
        Uv(1:2,2)=[Uv(2) Uv(2)];
        Uv(1:2,1)=[Uv(1,1) Uv(1,1)];
        points(1:iterations+2,3)=HeadingStart;
        %points=[]; %nem létezik út
%         save;
end

function [Openset, Closedset, n,m]=updateNeighbours(Cones,delta,Openset,Closedset,CurrentPoint,n,m,HeadingStart)
deltaSize=size(delta);
for i=1:deltaSize(1)
   %chordLength=1;
   NextPoint=struct('discpos',cell(1),'heading',cell(1),'x',cell(1),'uv',cell(1),'ud',cell(1),'g',...
    cell(1),'f',cell(1),'np',cell(1));
   if abs(CurrentPoint.ud+delta(i))<pi/2
       NextPoint.ud=CurrentPoint.ud+delta(i);
   else
       NextPoint.ud=sign(CurrentPoint.ud+delta(i))*pi/2;
   end
   NextPoint.uv=max(min(2,CurrentPoint.uv+cos(NextPoint.ud*2)/10),2);
   NextPoint.heading=CurrentPoint.heading+NextPoint.uv*sin(NextPoint.ud)/2;
   NextPoint.x=[CurrentPoint.x(1)+CurrentPoint.uv*cos(NextPoint.heading),CurrentPoint.x(2)+CurrentPoint.uv*sin(NextPoint.heading)];
   NextPoint.discpos=cont2disc(NextPoint.x);
   NextPoint.np=CurrentPoint.discpos;
   ClosedsetSize=size(Closedset');
   for j=1:ClosedsetSize(1)
       if Closedset(j).discpos==NextPoint.discpos
           break
       end
   end
%    ContFromDiscPos=disc2cont(NextPoint.discpos);
   if ~all(Closedset(j).discpos==NextPoint.discpos)
       if any(vecnorm((Cones-NextPoint.x)')<=0.6)
           n=n+1;
           Closedset(n)=NextPoint;
       else
           
           %calculating NextPoint.g

           NextPoint.g=CurrentPoint.g+1;
%            if NextPoint.g==3
%                j=j;
%            end
           %and NextPoint.f
           NextPoint.f=Heuristics(NextPoint.x,Cones,NextPoint.heading,CurrentPoint.heading);
           
           
           %if smaller replace it
           OpensetSize=size(Openset');
           for j=1:OpensetSize(1)
               if ~isempty(Openset(j).discpos) && all(Openset(j).discpos==NextPoint.discpos)
                   break
               end
           
           end
           if ~isempty(Openset(j).discpos) && all(Openset(j).discpos==NextPoint.discpos)
               
               if NextPoint.g <Openset(j).g
                   Openset(j)=NextPoint;
               end
           else
           m=m+1;
           Openset(m)=NextPoint;
           end
       end
    end
end
end