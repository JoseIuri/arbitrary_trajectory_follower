function zp = sgorbissa(t,z)
	%state
	x=z(1,1);
	y=z(2,1);
	q=z(3,1);
	qc=z(4,1);
	%
	f=(x+2)^2 − y;
	fx= 2*(x+2);
	fxx=2;
	fxy=0;
	fy=−1;
	fyy=0;
	%
	Vf=sqrt(fx^2+fy^2);
	K2=1.0;
	S=K2*f/sqrt(1+f^2);
	%
	u=0.5;
	dqc=((fx*fxy−fy*fxx)*u*cos(q)+(fx*fyy−fy*fxy)*u*sin(q))/Vf/Vf;
	K1=2;
	r=K1*(−Vf*u*S−fx*abs(u)*cos(q)−fy*abs(u)*sin(q))+dqc;
	%
	zp(1,1)=u*cos(q);
	zp(2,1)=u*sin(q);
	zp(3,1)=r;
	zp(4,1)=dqc;
end