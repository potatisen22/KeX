clc;
clear all;
close all;
%% Linear state space equation dx = Ax + Bu
% x = [phi theta psi p q r u v w x y z]^T
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

%% LQR control
%Constants, G(s)*U(s) = Y(s), R(s)-Y(s) = E(s)  
%xvector is input from modell and potentialfields
% for i = 1:12
%    Rs(i) = laplace(x(i)) 
%     
% end
C = eye(12);
D = 0;
Ts = 0.05;
q = 50; %gain
Q = (C'*C)*q;
R = eye(4);
[Kd,S,e] = lqrd(A,B,Q,R,Ts); 
Ac = [(A-B*Kd)];
Bc = [B];
Cc = [C];
Dc = [D];
sys_cl = ss(Ac,Bc,Cc,Dc);
x0 = ones(1,12);  % initial state
for i = 1:12
    figure(i)
    initial(sys_cl(i,:),x0);
    grid on
end
S = stepinfo(sys_cl)

 %H = tf(sys_cl);
% clf
% for i = 1:12
%     t = 0:0.01:4;
%     u = sin(10*t);
%     figure(i)
%     %lsim(H(i,1)+H(i,2)+H(i,3)+H(i,4),u,t)
%     step(H(i,1)+H(i,2)+H(i,3)+H(i,4))
%     hold on
% end
% for i = 1:12
%     Gcl = H(i,1)+H(i,2)+H(i,3)+H(i,4);
%     Es(i) = Rs(i) - Ys(i)
%     Ys(i)=Rs(i)*Gcl;
% end
% output = ilaplace(Ys)
