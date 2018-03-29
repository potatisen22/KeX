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


%Printing initial position and goal
scatter3(qp(1),qp(2), qp(3))
hold on
scatter3(qgoal(1),qgoal(2), qgoal(3))
hold on

xi = 1; %scale factor for attractive potential
eta = 0.5; %scale factor for repulsive potential
p0 = 2; %radius of sphere of influence for repulsive potential

%Generating random # of obstacles between min and max
min = 100;
max= 300;
[totalobst,obstpos] = obstaclegenerator(min,max);


Fdir = [0,0,0]; %Force matrix with all force vectors on each time step
v0 = 0; %Initial velocity
rhogoal = norm(qp-qgoal); %Distance to goal
while rhogoal > 0.1

    %Attractive force and potential calculation
    [Uatt, Fatt] = attractive(d, xi, qp, qgoal, rhogoal);
    
    %Repulsive force and potential calculation
    [Ureptotstatic, Freptotstatic] = repulsive(p0, eta, qp, totalobst, obstpos);
    
    %Total force and potential -> attractive + repulsive
    U = Uatt+Ureptotstatic;
    F = Fatt+Freptotstatic;
    
    %Moving vehicle according to the force vector obtained
    Fdir = [Fdir; F/norm(F)];
    vvec = F.*tstep %+ v0;
    qp = F*tstep + qp;
    scatter3(qp(1),qp(2),qp(3), 3,'b')
    hold on
    rhogoal = norm(qp-qgoal) %Updating distance to goal
    pause(0.01)
    %v0 = norm(vvec)
end