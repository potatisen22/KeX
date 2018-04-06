%TO DO LIST
%DONE repulsive surface with other drones too
%implement ODE
%DONE make it so obstacles can't appear near initial position/goal of drones
%method to find if there's a collision or not

%for the report
%plot distances to the goal (t)
%plot distances between drones -> they should always be positive
%test local minima -> do a symetrial case and show it in the report -> cite
%works that try to solve it as references

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
eta = 5; %scale factor for repulsive potential
p0 = 2; %radius of sphere of influence for repulsive potential

%Generating random # of drones (initial pos+goal) between min and max
min = 3;
max = 7;
raddrones = 0.3;
[totaldrones, drones, goaldrones, colordrones] = uavgenerator3d (min, max, raddrones);

%Generating random # of obstacles between min and max
%The obstacles can't be in the same position as the drones' initial
%position or their goals
%Currently this is solved because obstacles can only appear between 1 and 9
%and drones/goals between 0 and 1, and 9 and 10 respectively
min = 50;
max= 100;
[totalobst, obstpos, obstrad] = obstaclegenerator3d(min, max, totaldrones, drones, goaldrones, raddrones);

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
            [Ureptotstatic, Freptotstatic] = repulsivesurface(p0, eta, drones(i,:), totalobst, obstpos, obstrad);
            
            %Dynamic repulsive force and potential calculation
            totalobstdrones = totaldrones-1;
            obstdronespos = drones([1:i-1,i+1:totaldrones],:);
            [Ureptotdynamic, Freptotdynamic] = repulsivesurface(p0, eta, drones(i,:), totalobstdrones, obstdronespos, raddrones*ones(totalobstdrones,1));
            
            %Total force and potential -> attractive + repulsive
            U = Uatt + Ureptotstatic + Ureptotdynamic;
            F = Fatt + Freptotstatic + Freptotdynamic;

            %Moving vehicle according to the force vector obtained
            vvec = F.*tstep;
            drones(i,:) = F*tstep + drones(i,:);
            printspherecolor(drones(i,:), raddrones, colordrones(i,:))
        end
    end
    pause(0.01)
end