%for the report
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
xi = 10; %scale factor for attractive potential
eta = 10; %scale factor for repulsive potential
p0 = 2;%radius of sphere of influence for repulsive potential

%Figures to plot
%Real time environment with drones flying to goals
%Figure 1
figure('Name','Environment','NumberTitle','off')

%Distance from drone surface to goal
%Figure 2
figure('Name','Distance from drone surface to goal (time)','NumberTitle','off')
grid on

%Distance of drones to closest static or dynamic obstacle, taking radius into account
%Figure 3
figure('Name','Distance from drone surface to obstacle surface (time)','NumberTitle','off')
grid on

%Drones' velocity
%Figure 4
figure('Name','Drones velocity (time)','NumberTitle','off')
grid on


%Generating random # of drones (initial pos+goal) between min and max
mindrones = 3;
maxdrones = 7;
raddrones = 0.3;
figure(1)
[totaldrones, drones, goaldrones, colordrones] = uavgenerator2d(mindrones, maxdrones, raddrones);

%Generating random # of obstacles between min and max
minobst = 10;
maxobst = 25;
figure(1)
[totalobst, obstpos, obstrad] = obstaclegenerator2d(minobst, maxobst, totaldrones, drones, goaldrones, raddrones);

%Multi drone avoidance
% raddrones = single(0.3);
% totaldrones = single(3);
% drones = single([0, 0, 0;
%           3, 3, 3,
%           0, 3, 0]);
% goaldrones = single([2.8, 2.8, 2;
%               0, 0, 0.5,
%               3, 0, 1.5]);
% colordrones = single([0.1, 0.1, 0.1;
%                0.9, 0.9, 0.9;
%                0.5, 0.2, 0.1]);
% totalobst = single(1); 
% obstpos = single([7, 7, 7]);
% obstrad = single(0.4);

%Local minima problem with drone and obstacle
% raddrones = single(0.3);
% totaldrones = single(1);
% drones = single([0, 0, 0]);
% goaldrones = single([5, 5, 5]);
% colordrones = single([0.4, 0.2, 0.7]);
% figure(1)
% printspherecolor(drones, raddrones, colordrones)
% scatter3(goaldrones(1,1),goaldrones(1,2),goaldrones(1,3),3,colordrones)
% hold on
% totalobst = single(1); 
% obstpos = single([2.5, 2.5, 2.5]);
% obstrad = single(1);
% printsphere(obstpos, obstrad)

%Local minima problem with three drones
% raddrones = single(0.3);
% totaldrones = single(3);
% drones = single([0, 0, 0;
%           3, 3, 3,
%           1, 1, 1]);
% goaldrones = single([3, 3, 3;
%               0, 0, 0.5,
%               3, 0, 1.5]);
% colordrones = single([0.1, 0.1, 0.1;
%                0.9, 0.9, 0.9;
%                0.5, 0.2, 0.1]);
% totalobst = single(1); 
% obstpos = single([7, 7, 7]);
% obstrad = single(0.4);

finished = single(zeros(totaldrones, 1));
completed = single(ones(totaldrones,1));
iteration=1;
while(~ isequal(finished,completed))
    for i=1:totaldrones
        rhogoal = single(norm(drones(i,:)-goaldrones(i,:)));
        if(rhogoal < 0.1)
            finished(i,1) = 1;
        else
            %Attractive force and potential calculation
            [Uatt, Fatt] = attractive(d, xi, drones(i,:), goaldrones(i,:), rhogoal);

            %Static repulsive force and potential calculation
            [Ureptotstatic, Freptotstatic, closestdiststat] = repulsivesurface(p0, eta, drones(i,:), raddrones, totalobst, obstpos, obstrad);
            
            %Dynamic repulsive force and potential calculation
            totalobstdrones = totaldrones-1;
            obstdronespos = drones([1:i-1,i+1:totaldrones],:);
            [Ureptotdynamic, Freptotdynamic, closestdistdynam] = repulsivesurface(p0, eta, drones(i,:), raddrones, totalobstdrones, obstdronespos, raddrones*ones(totalobstdrones,1));
            
            %Total force and potential -> attractive + repulsive
            U = Uatt + Ureptotstatic + Ureptotdynamic;
            F = Fatt + Freptotstatic + Freptotdynamic;

            %Moving vehicle according to the force vector obtained
            vvec = F.*tstep;
            drones(i,:) = F*tstep + drones(i,:);
            figure(1)
            printcirclecolor(drones(i,:), raddrones, colordrones(i,:))
            
            %Printing distance to goal from the surface of the drone
            disttogoal = norm(drones(i,:)-goaldrones(i,:)-raddrones);
            figure(2)
            scatter(iteration*tstep, disttogoal, 10, colordrones(i,:))
            hold on
            grid on
            
            %Printing distance to closest obstacle, dynamic or static
            closestdist = min(closestdiststat, closestdistdynam)- raddrones;
            figure(3)
            scatter(iteration*tstep, closestdist, 10, colordrones(i,:))
            hold on
            grid on
            
            %Printing velocity (norm)
            vel = norm(vvec);
            figure(4)
            scatter(iteration*tstep, vel, 10, colordrones(i,:))
            hold on
            grid on
        end
    end
    pause(0.01)
    iteration = iteration + 1 ;
end