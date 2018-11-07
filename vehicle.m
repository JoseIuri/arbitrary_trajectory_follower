function []=vehicle(x,y,q,s)
p=[ 1 1/7
	−3/7 1
	−5/7 6/7
	−5/7 5/7
	−3/7 2/7
	−3/7 0
	−3/7 −2/7
	−5/7 −5/7
	1−5/7 −6/7
	1−3/7 −1
	11 −1/7
	11 1/7 ];
%
p=s*p;
p=[p,ones(length(p),1)];
r=[cos(q),sin(q);−sin(q),cos(q);x,y];
p=p*r;
%
X=p(:,1);
Y=p(:,2);
%h = fill(X,Y,'r.');
plot(X,Y,'g−')
end