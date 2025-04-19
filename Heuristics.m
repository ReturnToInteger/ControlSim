function heur=Heuristics(point, Cones,a,alphaPrev)
param1=50;
param2=30;
param3=2;
param4=40;
param5=6;
% param6=100;


rel=(Cones-point);

% Transzformálás
for i=1:size(Cones,1)
    rel(i,:)=rel(i,:)*[cos(a) -sin(a);sin(a) cos(a)];
end

k=rel(:,1)>=0;

l=(vecnorm(rel(:,:)')<param5)';


% Cone felbontás, súlyozás
if any(k)
    normsum=sum(rel(k,:).^2/(sum(rel(k,:))),1);
    minx=mink(sum(rel(k,:).^2,2),2);
else
    minx=Inf;
    normsum=Inf;
end
minneg=mink(sum(rel(~k,:).^2,2),2);


heur=param3^(sum(normsum/param4)+sum(minx/param2)+sum(minneg/param1));

if isnan(heur)
    heur=Inf;
end



