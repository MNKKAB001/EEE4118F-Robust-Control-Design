s = tf('s'); %define Laplace variable as s
P = s*zeros(1,1,5); %initialise P
i = 0;
tau=[0.473 0.48 0.099 0.17 2.15 2.37];
for k=[6.15 11.69 1.1 1.83 7.39 11.14] %generate plant set with nominal
plant in first index
i = i+1;
tau2 = tau(i);
% P(1,1,i) = ((1-0.1*s/2)*k)/(s*tau2+1); %Velocity loop plant
P(1,1,i) = k/(s*tau2+1); %Velocity loop plant
end

%%Speed Loop
%Plant templates
wTemp = [.83,15,100];
disp(' ')
disp('plottmpl(w,P); %show templates')
drawnow
plottmpl(wTemp,P), title('Plant Templates')
% %Calculations
% % Velocity loop output
VelocityOut= tf(4.26,[1,1.67,4.26]);
% Velocity loop input error
VelocityError = tf([1,1.67,0],[1,1.67,4.26]);
% Velocity loop output analysis

figure(2)
subplot(2,1,1);
stepplot(VelocityOut,'r',5);
ylabel('T_Yv_/_Rv')
title('Desired velocity closed loop step response envelope')
subplot(2,1,2);
ww=logspace(-1,2);
TTup=abs(freqcp(4.26,[1,1.67,4.26],ww)); % upper bound
semilogx(ww,db(TTup),'k')
axis([0.1,100,-40,5])
grid
title('Desired velocity closed loop Bode magnitude envelope')
ylabel('T_Yv_/_Rv [dB]')
xlabel('frequency [rad/s]')
% % Velocity loop input error analysis

figure(3)
subplot(2,1,1);
stepplot(VelocityError,'r',5);
ylabel('T_Ev_/_Rv')
title('Velocity input error closed loop step response envelope')
subplot(2,1,2);
TTerror=abs(freqcp([1,1.67,0],[1,1.67,4.26],ww)); % upper bound
semilogx(ww,db(TTerror),'k')
axis([0.1,100,-40,5])
grid
title('Velocity input error closed loop Bode magnitude envelope')
ylabel('T_Ev_/_Rv [dB]')
xlabel('frequency [rad/s]')
% %Bounds

% Sensitivity
W=[0.1,0.25,1,3,8,10,100,1000]; %set of discrete design frequency points
b1=sisobnds(2,W,10^(-20/20),P); % |1/(1+L)| <=-20dB, w<=0.4095rad/s
b2=sisobnds(2,W,10^(3/20),P); %|1/(1+L)| <= 3dB, \forall w
bnd=grpbnds(b1,b2); % group the bounds for use in lpshape

% Controller
Gvelocity = tf(1,[1 0]);
lpshape(W,bnd,P(1,1,1),Gvelocity) % CAD tool for loop-shaping
%This code is for the position loop controller design first run the
%velocity loop before this code.
s = tf('s'); %define Laplace variable as s
Ppos = s*zeros(1,1,5); %initialise P
i = 0;
tau=[0.473 0.48 0.099 0.17 2.15 2.37];
A=[6.15 11.69 1.1 1.83 7.39 11.14];
GpositionDig =1;
Gvelocity=10;
for newA = Gvelocity.*A %generate plant set with nominal plant in first
index
i = i+1;
tau2 = tau(i);
Ppos(1,1,i) = ((1-0.1*s/2)*newA)/(30*s*(s*tau2+1+newA));
end
% %Bounds

figure(1)
subplot(2,1,1)
W=[0.1,0.3,0.4095,1.099,3,8,10,30];
ww=logspace(-1,2);
TFup=tf([0.02735 0.8206],[1,1,0.8206]); % upper bound
TFlow=tf(0.1296,conv([1,1.2,0.36],[1 0.36]));% upper bound
step(tf([0.02735 0.8206],[1,1,0.8206]),'k',tf(0.1296,conv([1,1.2,0.36],[1
0.36])),'k');
grid
title('Desired closed loop step response envelopee')
ylabel('T_Y_/_R')

%Bode Plots
% For L/1+L
subplot(2,1,2)
semilogx(ww,db([TFup;TFlow]),'k');
axis([0.1,100,-40,5])
grid
title('Desired closed loop Bode magnitude envelope')
ylabel('T_Y_/_R [dB]')
xlabel('frequency [rad/s]')
figure(1)
bodemag(Ppos,TFup1,'r--',TFlow1,'b',ww);
grid
%Bounds

b1=sisobnds(7,W,[TFup;TFlow],Ppos);
b2=sisobnds(2,W,10^(3/20),Ppos);
bnd=grpbnds(b1,b2);

% %Controller
Gposition1=tf(1,[1 0]);
lpshape(W,bnd,Ppos(1,1,1),Gposition1)
grid
preFilter1=tf(1,[0 1]);
pfshape(7,ww,[TFup;TFlow],Ppos,[],Gposition,[],preFilter1)
grid
% This is a code for the single loop controller design
s = tf('s'); %define Laplace variable as s
PsingleLoop = s*zeros(1,1,5); %initialise P
i = 0;
tau=[0.473 0.48 0.099 0.17 2.15 2.37];
A=[6.15 11.69 1.1 1.83 7.39 11.14];
for newA = (1/30)*A %generate plant set with nominal plant in first index
i = i+1;
tau2 = tau(i);
PsingleLoop(1,1,i) = ((1-0.1*s/2)*newA)/(s*(s*tau2+1));
end

% Calculations
% Velocity loop output
VelocityOut= tf(2.86,[1,2,2.86]);
% Velocity loop input error
LoopError = tf([1,1.67,0],[1,1.67,3.34]);
% Velocity loop input error analysis
figure(1)
subplot(2,1,1);
stepplot(LoopError,'r',5);
ylabel('T_Ev_/_Rv')
title('Error closed loop step response envelope')
%
subplot(2,1,2);
ww=logspace(-1,2);
LoopTTerror=abs(freqcp([1,1.67,0],[1,1.67,3.34],ww)); % upper bound
semilogx(ww,db(LoopTTerror),'k')
axis([0.1,100,-40,5])
grid
title('Error closed loop Bode magnitude envelope')
ylabel('T_Ev_/_Rv [dB]')
xlabel('frequency [rad/s]')
% %Bounds

W=[0.1,0.3,0.4095,1.099,3,8,10,30,50];
TupLoop=abs(freqcp([0.0 3.34],[1,5/3,3.34],ww)); % upper bound
TloLoop=abs(freqcp(0.9,[1,2,0.9],ww));
TFupLoop=tf([0.0835 3.34],[1,5/3,3.34]);
TFlowLoop=tf(0.2,conv([1,2,0.9],[1 0.36])); % lower bound
figure(1)
subplot(2,1,1)
step(tf([0.0 3.34],[1,5/3,3.34]),'k',tf(0.9,[1,2,0.9]),'k');
grid
title('Desired closed loop step response envelopee')
ylabel('T_Y_/_R')
% Bode Plots
% For L/1+L

subplot(2,1,2)
semilogx(ww,db([TupLoop;TloLoop]),'k');
axis([0.1,100,-40,5])
grid
title('Desired closed loop Bode magnitude envelope')
ylabel('T_Y_/_R [dB]')
xlabel('frequency [rad/s]')
figure(1)
bodemag(PsingleLoop,TFupLoop,'r--',TFlowLoop,'r--',ww);
grid
%Bounds
b1=sisobnds(7,W,[TFupLoop;TFlowLoop],PsingleLoop);
b2=sisobnds(2,W,10^(4.284/20),PsingleLoop);
bnd=grpbnds(b1,b2);

% Controller
% obtained GsingleLoop =(s+0.485)*(s+0.2996)/(s*(s+55.67));
GsingleLoop = 1/s;
lpshape(W,bnd,PsingleLoop(1,1,1),GsingleLoop)
grid
% obtained preFilterLoop=tf(0.9098,conv([1 1.532],[1 0.6168]));
preFilterLoop = tf([0 1],[0 1]);
pfshape(7,ww,[TFup;TFlow],PsingleLoop,[],GsingleLoop,[],preFilterLoop)
grid