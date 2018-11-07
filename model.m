clear all
close all
clc
%Experiment 1
y0 = [3,5,0,0]';
y = y0(:);
%
ti = 0;
tf = 50;
tspan = ti:0.05:tf;

%
options = odeset('RelTol',1e−4,'AbsTol',1e−4*ones(size(y0)));
tic
[T,Y] = ode45(@(t,y)sgorbissa(t,y),tspan,y0,options);
toc
%
Xm = Y(:,1);
Ym = Y(:,2);
Qm = Y(:,3);
 %
 figure(1)
 h = plot(T,Xm,'r',T,Ym,'g',T,Qm,'b');
 set(h,'linewidth',1.5);
 xlabel('Time (s)');
 legend('x_{m}','y_{m}','\theta_{m}')
 grid on
 %
 figure(2)
 if (~ishold),
 	hold on
 end
 axis('equal')
 h=plot(Xm,(Xm+2).^2);
 set(h,'linewidth',1.5);
 xlabel('x_{m}');
 ylabel('y_{m}');
 grid on

 for i=1:25:length(Xm),
 	vehicle(Xm(i),Ym(i),Qm(i),0.2);
 end

 Xp = [diff(Xm); 0];
 Yp = [diff(Ym); 0];
 Qp = [diff(Qm); 0];