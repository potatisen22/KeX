%add radius to everything
%add sveral obstacle and goals
%implement ODE
clc;
clear all;
close all;
tstep = 0.05; %time step
v = 5;
x = 0;
y = 0;
z = 0;
d = 0.5; %distance to change from conic to parabolic well
qgoal= [10,10,10]; %in earth frame
qp = [x,y,z]; %should import form modell, current position
rhogoal = norm(qp-qgoal); %distance to goal
xi = 1; %scale factor for attractive potential
eta = 0.5; %scale factor for repulsive potential
p0 = 2; %radius of sphere of influense for repulsive potential
objpos =[1.1,1,1]; %obstacleposition
scatter(objpos(1),objpos(2),20,'*r')
hold on
scatter(qp(1),qp(2))
hold on
scatter(qgoal(1),qgoal(2))
Fdir = [0,0,0];
v0 = 0
while rhogoal > 0.1
    rhogoal = norm(qp-qgoal);
    gradrhogoal = (qp-qgoal)/rhogoal; %gradient of rhogoal
    if rhogoal > d
        Uatt = (1/2*xi)*rhogoal.^2; %attractive potential if we are far from goal
        Fatt = -xi*(qp-qgoal)/rhogoal;
    else
        Uatt =  d*xi*rhogoal; %attractive potential if we are close to goal.
        Fatt = -d*xi*(qp-qgoal)/rhogoal;
    end
    rhoq = norm(qp-objpos);%distance to obstacles
    if rhoq < p0 %try 1/rho, rho = dist^2
       Urep = 1/2*eta*(1/rhoq-1/p0)^2;
       Frep =  eta*(1/rhoq-1/p0)*(1/rhoq^2)*((qp-objpos)/norm(qp-objpos));%hmm this one?
    else 
        Urep = 0;
        Frep = 0;
    end
    U = Uatt+Urep;
    F = Fatt+Frep;
    %Fdir = [Fdir; F/norm(F)];
    vvec = F.*tstep + v0;
    qp = F*tstep + qp;
    scatter(qp(1),qp(2),3,'b')
    hold on
    rhogoal = norm(qp-qgoal) %distance to goal
    v0 = norm(vvec);
end