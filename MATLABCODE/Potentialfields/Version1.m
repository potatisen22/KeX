clc;
clear all;
close all;
tstep = 0.01 %time step
v = 0;
x = 0;
y = 0;
z = 0;
d = 1 %distance to change from conic to parabolic well
qgoal= [3,3,3]; %in earth frame
qp = [x,y,z]; %should import form modell, current position
rhogoal = norm(qp-qgoal); %distance to goal
while rhogoal > 0.1 
    gradrhogoal = (qp-qgoal)/rhogoal; %gradient of rhogoal
    xi = 2; %scale factor for attractive potential
    if rhogoal > d
        Uatt = (1/2*xi)*rhogoal.^2; %attractive potential if we are far from goal
        Fatt = -d*xi*(qp-qgoal)/rhogoal;
    else
        Uatt =  d*xi*rhogoal; %attractive potential if we are close to goal.
        Fatt = d*xi*rhogoal;
    end
    eta = 2; %scale factor for repulsive potential
    p0 = 2; %sphere of influense for repulsive potential
    objpos =[2,1,2]; %obstacleposition
    rhoq = norm(qp-objpos)%distance to obstacles
    if rhoq < p0
       Urep = 1/2*eta*(1/rhoq-1/p0);
       Frep =  eta*(1/rhoq-1/p0)*(1/rhoq^2)*((qp-objpos)/abs(qp-objpos));%hmm this one?
    else 
        Urep = 0;
        Frep = 0;
    end
    U = Uatt+Urep;
    F = Fatt+Frep;
    v = F*tstep + v
    qp = v*tstep + qp;
    rhogoal = norm(qp-qgoal); %distance to goal
end