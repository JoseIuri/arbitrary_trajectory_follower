%% P3DX dimensions
D = 0.195;
R = D/2;
L = 0.038;
Vmax = 1.2;

%% Running V−REP and loading test scene
%%winopen('cena.ttt');
%% Loading V−REP remote interface − client side
vrep = remApi('remoteApi');
%% Closing any previously opened connections
vrep.simxFinish(−1);
%% Connecting to remote V−REP API server
retCod = 0;
connectionAddress = '127.0.0.1';
Capítulo 4. Conclusão
connectionPort = 19997;
waitUntilConnected = true;
doNotReconnectOnceDisconnected = true;
timeOutInMs = 5000;
commThreadCycleInMs = 5;
while(retCod == 0)
	[clientID]=vrep.simxStart(connectionAddress,connectionPort,waitUntilConnected,doNotReconnectOnceDisconnected,timeOutInMs,commThreadCycleInMs);
	if(clientID > −1),
		fprintf('Starting\n');
		retCod = 1;
	else
		fprintf ('Waiting\n');
	end
end

%% Getting robot handles
[retCod,rob1] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait);
[retCod,rob1LM] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait);
[retCod,rob1RM] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait);

%% Defining robot initial position
% Position
xp0 = y(1);
yp0 = y(2);
zp0 = R + 0.1;
% Orientation
ap0 = 0;
bp0 = 0;
cp0 = y(3);
[retCod] = vrep.simxSetObjectPosition(clientID,rob1,−1,[xp0,yp0,zp0],vrep.simx_opmode_oneshot);
[retCod] = vrep.simxSetObjectOrientation(clientID,rob1,−1,[ap0,bp0,cp0],vrep.simx_opmode_oneshot);
%% Starting V−REP simulation
[retCod] = vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
%% Defining V−REP client side controller parameters
np = length(T);
hd = 50e−3;
%tf = 50;
tf = (np−1)*hd;
tc = 0;
td = 0;
%
Ups = Vmax/4;
OmeMax = 1.00;
fd = 1/2;
%
t = zeros(np,1);
xp = zeros(np,1);
yp = zeros(np,1);
fp = zeros(np,1);
vp = zeros(np,1);
wp = zeros(np,1);
%
id = 1;
%
Kx = 1.5;
Ky = 8;
Ka = 140;

%% Main control loop − V−REP client side
t0 = cputime
while tc < tf,
	tc = cputime − t0;
%% Current sampling instant
	if tc > td,
		t(id) = tc;
		%% Measuring and saving
		[retCod,rob1Pos] = vrep.simxGetObjectPosition(clientID,rob1,−1,vrep.simx_opmode_oneshot_wait);
		[retCod,rob1Ori] = vrep.simxGetObjectOrientation(clientID,rob1,−1,vrep.simx_opmode_oneshot_wait);
		% Robot pose
		xa = rob1Pos(1,1);
		ya = rob1Pos(1,2);
		fa = rob1Ori(1,3);
		xp(id) = xa;
		yp(id) = ya;
		fp(id) = fa;
		%% Controlling
		ex = (cos(fa)*(Xm(id)−xa)+sin(fa)*(Ym(id)−ya));
		ey = (−sin(fa)*(Xm(id)−xa)+cos(fa)*(Ym(id)−ya));
		ea = Qm(id)−fa;
		Vr = sqrt((Xp(id)^2)+(Yp(id)^2));
		Wr = Qp(id);
		%Vd = Vr;
		Vd = Vr*cos(ea)+Kx*ex;
		%Wd = Qp(id);
		Wd = Wr + Vr*(Ky*ey+Ka*sin(ea));

		% Differential velocities
		leftVel=(Vd − L*Wd)/R;
		rightVel=(Vd + L*Wd)/R;
		%% Actuating
		[retCod] = vrep.simxSetJointTargetVelocity(clientID,rob1LM,leftVel,vrep.simx_opmode_oneshot);
		[retCod] = vrep.simxSetJointTargetVelocity(clientID,rob1RM,rightVel,vrep.simx_opmode_oneshot);
		%% Next sampling instant
		td = td + hd;
		id = id + 1;
		%td = T(id);
	end
end

%% Stoping V−REP simulation
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
fprintf('Ending\n');
%
vrep.simxFinish(clientID);
vrep.delete();
%% Plotting results
figure(3)
plot(t,xp,t,yp,t,fp),grid
legend('x_p','y_p','\phi_p')
xlabel('t [s]')
% 
figure(4)
hold on

axis('equal')
h=plot(Xm,(Xm+2).^2);
set(h,'linewidth',1.5);
xlabel('x_{m}');
ylabel('y_{m}');
grid on
plot(xp,yp,'r−'),grid

legend('x_p\times y_p')
%% Stopping V−REP
%dos('taskkill /F /IM vrep.exe');