clear;
load TestTrack.mat


% VhclAbsPosy=([VhclAbsPosY(1); VhclAbsPosY(1:end-1)]...
%     -[VhclAbsPosY(2:end); VhclAbsPosY(end)]);


VhclAbsHead=atan((VhclAbsPosY([2:end, end])-VhclAbsPosY([1 1:end-1]))./(VhclAbsPosX([2:end, end])-VhclAbsPosX([1 1:end-1])));

k=VhclAbsHead>0 & VhclAbsPosX([2:end, end])-VhclAbsPosX([1 1:end-1])<=0 ...
    & VhclAbsPosY([2:end, end])-VhclAbsPosY([1 1:end-1])<0;
VhclAbsHead(k)=-pi+VhclAbsHead(k);

j=VhclAbsHead<0 & VhclAbsPosX([2:end, end])-VhclAbsPosX([1 1:end-1])<=0 ...
    & VhclAbsPosY([2:end, end])-VhclAbsPosY([1 1:end-1])>0;
VhclAbsHead(j)=pi+VhclAbsHead(j);

% l=VhclAbsHead>=0 & VhclAbsPosX([2:end, end])-VhclAbsPosX([1 1:end-1])<0;
% VhclAbsHead(l)=-pi+VhclAbsHead(l);
% 
% m=VhclAbsHead>0 & VhclAbsPosX([2:end, end])-VhclAbsPosX([1 1:end-1])<0;
% VhclAbsHead(m)=-pi+VhclAbsHead(m);

clear j k l m;

save TestTrack.mat