clc;
clear all;
close all;
%% Linear state space equation dx = Ax + Bu
%Constants 
g = 9.8; %Gravity
Ix=1; %Inertia x
Iy=1; %Intertia y
Iz=2; %Intertia z
m=3; %mass of the quadrotor

A = [0,0,0,1,0,0,0,0,0,0,0,0;
     0,0,0,0,1,0,0,0,0,0,0,0;
     0,0,0,0,0,1,0,0,0,0,0,0;
     0,0,0,0,0,0,0,0,0,0,0,0;
     0,0,0,0,0,0,0,0,0,0,0,0;
     0,0,0,0,0,0,0,0,0,0,0,0;
     0,g,0,0,0,0,0,0,0,0,0,0;
     -g,0,0,0,0,0,0,0,0,0,0,0;
     -g,0,0,0,0,0,0,0,0,0,0,0;
     0,0,0,0,0,0,1,0,0,0,0,0;
     0,0,0,0,0,0,0,1,0,0,0,0;
     0,0,0,0,0,0,0,0,1,0,0,0];

B = [0,0,0,0;
     0,0,0,0;
     0,0,0,0;
     0,1/Ix,0,0;
     0,0,1/Iy,0;
     0,0,0,1/Iz;
     0,0,0,0;
     0,0,0,0;
     1/m,0,0,0;
     0,0,0,0;
     0,0,0,0;
     0,0,0,0];
 
 C= [1,0,0,0,0,0,0,0,0,0,0,0;
     0,1,0,0,0,0,0,0,0,0,0,0;
     0,0,1,0,0,0,0,0,0,0,0,0;
     0,0,0,1,0,0,0,0,0,0,0,0;
     0,0,0,0,1,0,0,0,0,0,0,0;
     0,0,0,0,0,1,0,0,0,0,0,0;
     0,0,0,0,0,0,1,0,0,0,0,0;
     0,0,0,0,0,0,0,1,0,0,0,0;
     0,0,0,0,0,0,0,0,1,0,0,0;
     0,0,0,0,0,0,0,0,0,1,0,0;
     0,0,0,0,0,0,0,0,0,0,1,0;
     0,0,0,0,0,0,0,0,0,0,0,1];
 
 %% Checking observability and controllability
 %Building observability matrix
 Obs = [C];
 for i=1:11
 Obs = [Obs;(C*(A^i))]
 end
%Size needs to be 144x12
sizeObs = size(Obs)
%In order for the system to be observable, the controllability matrix
%needs to be full rank
rankObs = rank(Obs)

 %Building controllability matrix
 Cont = [B];
for i=1:11
 Cont = [Cont,((A^i)*B)];
end
%Size needs to be 12x48
sizeCont = size(Cont)
%In order for the system to be controllable, the controllability matrix
%needs to be full rank
rankCont = rank(Cont)

%The other way (easier) to check controllabillity and observabillity
Co = ctrb(A,B);
%Length of uncontrollable states, should be zero
unco = length(A) - rank(Co)

Ob = obsv(A,C);
%Length of unobservable states, should be zero
unob = length(A)-rank(Ob)

%% linear control, not working tried alot of different poles
%  D = 0;
%  sys = ss(A,B,C,D)
%  p = [-30+10*i -30-10*i -35+5*i -35-5*i -20+2*i -20-2*i -50+8*i -50-8*i -55 -70 -65 -90]%closed systems poles
%  q =[-1 -5 -8 -10 -13 -15 -20 -25 -30 -40 -50 -55] %pole placement for observer, should be far left b/c no error
%  K = place(A,B,p)
%  Ac = [(A-B*K)];
%  Bc = [B];
%  Cc = [C]%*ones(12,1);
%  Dc = [D];
%  sys_cl = ss(Ac,Bc,Cc',Dc)
%  H = tf(sys_cl);
%  
% %L = place(Ac',Cc',q)'
% %rsys = reg(sys,K,L)
%% LQR control
C = eye(12)
D = 0;
Ts = 0.1;
q = 50; %gain
Q = (C'*C)*q;
R = eye(4);
[Kd,S,e] = lqrd(A,B,Q,R,Ts); 
Ac = [(A-B*Kd)];
Bc = [B];
Cc = [C];
Dc = [D];
sys_cl = ss(Ac,Bc,Cc,Dc)
H = tf(sys_cl);
% for i = 1:12
%     step(H(i,1))
%     hold on
%     step(H(i,2))
%     hold on
%     step(H(i,3))
%     hold on
%     step(H(i,4))
%     hold on
% end
    %linearSystemAnalyzer(sys_cl)
