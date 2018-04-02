%add radius to everything
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

%Constants for all the vehicles
xi = 1; %scale factor for attractive potential
eta = 0.5; %scale factor for repulsive potential
p0 = 2; %radius of sphere of influence for repulsive potential

%Generating random # of drones (initial pos+goal) between min and max
min = 3;
max = 4;
[totaldrones, drones, goaldrones, colordrones] = uavgenerator (min, max);

%Generating random # of obstacles between min and max
%The obstacles can't be in the same position as the drones' initial
%position or their goals
%Currently this is solved because obstacles can only appear between 1 and 9
%and drones/goals between 0 and 1, and 9 and 10 respectively
min = 50;
max= 100;
[totalobst, obstpos] = obstaclegenerator(min, max, totaldrones, drones, goaldrones);

finished = zeros(totaldrones, 1);
completed = ones(totaldrones,1);
while(~ isequal(finished,completed))
    for i=1:totaldrones
        rhogoal = norm(drones(i,:)-goaldrones(i,:));
        if(rhogoal < 0.1)
            finished(i,1) = 1;
        else
            %Attractive force and potential calculation
            [Uatt, Fatt] = attractive(d, xi, drones(i,:), goaldrones(i,:), rhogoal);

            %Static repulsive force and potential calculation
            [Ureptotstatic, Freptotstatic] = repulsive(p0, eta, drones(i,:), totalobst, obstpos);
            
            %Dynamic repulsive force and potential calculation
            totalobstdrones = totaldrones-1;
            obstdronespos = drones([1:i-1,i+1:totaldrones],:);
            [Ureptotdynamic, Freptotdynamic] = repulsive(p0, eta, drones(i,:), totalobstdrones, obstdronespos);
            
            %Total force and potential -> attractive + repulsive
            U = Uatt + Ureptotstatic + Ureptotdynamic;
            F = Fatt + Freptotstatic + Freptotdynamic;

            %Moving vehicle according to the force vector obtained
            vvec = F.*tstep;
            drones(i,:) = F*tstep + drones(i,:);
            
            scatter3(drones(i,1),drones(i,2),drones(i,3),3,colordrones(i,:))
            hold on
        end
    end
    pause(0.001)
end