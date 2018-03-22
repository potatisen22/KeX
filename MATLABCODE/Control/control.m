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
     0,0,0,0,0,0,0,0,0,0,0,0;
     0,0,0,0,0,0,0,0,0,0,0,0;
     0,0,0,0,0,0,0,0,0,0,0,0;
     0,0,0,0,0,0,0,0,0,0,0,0;
     0,0,0,0,0,0,0,0,0,0,0,0;
     0,0,0,0,0,0,0,0,0,0,0,0;
     0,0,0,0,0,0,0,0,0,0,0,0;
     0,0,0,0,0,0,0,0,0,0,0,0;
     0,0,0,0,0,0,0,0,0,0,0,1];
 
 %% Checking observability and controllability
 %Building observability matrix
 Obs = [C];
 for i=1:11
 Obs = [Obs;(C*(A.^i))]
 end
%Size needs to be 144x12
sizeObs = size(Obs)
%In order for the system to be observable, the controllability matrix
%needs to be full rank
rankObs = rank(Obs)

 %Building controllability matrix
 Cont = [B];
for i=1:11
 Cont = [Cont,((A.^i)*B)];
end
%Size needs to be 12x48
sizeCont = size(Cont)
%In order for the system to be controllable, the controllability matrix
%needs to be full rank
rankCont = rank(Cont)

%% LQR control
%Q = 
%R = 
%[K,S,e] = lqr(A,B,Q,R) 

