A = repmat([1, 2],100,1);


param=sum(A,2)>1;
tic   
G=A(param,:);

for i=1:1000000
    G2=G;
end
toc