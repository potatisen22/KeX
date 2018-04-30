%test local minima -> do a symetrial case and show it in the report -> cite
%works that try to solve it as references

clc;
clear all;
close all;
tstep = 0.1; %time step
d = 1; %distance to change from conic to parabolic well

%Video
% environment_video_1 = VideoWriter('environment_1.avi'); 
% environment_video_1.FrameRate = 60;
% open(environment_video_1); 
% 
% environment_video_2 = VideoWriter('environment_2.avi'); 
% environment_video_2.FrameRate = 60;
% open(environment_video_2);
% 
% environment_video_3 = VideoWriter('environment_3.avi'); 
% environment_video_3.FrameRate = 60;
% open(environment_video_3); 

%Constants for all the vehicles
xi1 = 1; %scale factor for attractive potential
xi2 = 0.3;
eta = 1; %scale factor for repulsive potential
p0 = 2; %radius of sphere of influence for repulsive potential

%Figures to plot
%Real time environment with drones flying to goals
%Figure 1
figure('Name','Environment','NumberTitle','off')
xlabel('x / m')
ylabel('y /m') 
ylabel('z /m')

%Distance from drone surface to goal
%Figure 2
figure('Name','Distance from drone surface to goal (time)','NumberTitle','off')
grid on

%Distance of drones to closest static or dynamic obstacle, taking radius into account
%Figure 3
figure('Name','Distance from drone surface to obstacle surface (time)','NumberTitle','off')
grid on

%Velocity (norm)
%Figure 4
figure('Name','Norm of velocity (time)','NumberTitle','off')
grid on

%Generating random # of drones (initial pos+goal) between min and max
% mindrones = 5;
% maxdrones = 7;
% raddrones = 0.25;
% figure(1)
% [totaldrones, drones, goaldrones, colordrones] = uavgenerator3d (mindrones, maxdrones, raddrones);
% 
% %Generating random # of obstacles between min and max
% minobst = 50;
% maxobst = 100;
% figure(1)
% [totalobst, obstpos, obstrad] = obstaclegenerator3d(minobst, maxobst, totaldrones, drones, goaldrones, raddrones);

%Multi drone avoidance
% raddrones = single(0.3);
% totaldrones = single(3);
% drones = single([0, 0, 0;
%           6, 6, 6,
%           0, 6, 0]);
% goaldrones = single([5.6, 5.6, 4;
%               0, 0, 1,
%               5, 0, 3]);
% colordrones = single([0.3, 0.5, 0.15;
%                0.3, 0.2, 0.6;
%                0.2, 0.8, 0.1]);
% totalobst = single(1); 
% obstpos = single([7, 7, 7]);
% obstrad = single(0.4);

%Local minima problem with drone and obstacle
raddrones = single(0.3);
totaldrones = single(1);
drones = single([0, 0, 0]);
goaldrones = single([10, 10, 10]);
colordrones = single([0.4, 0.2, 0.7]);
figure(1)
printspherecolor(drones, raddrones, colordrones)
scatter3(goaldrones(1,1),goaldrones(1,2),goaldrones(1,3), 60, colordrones, 'filled', 'd')
hold on
totalobst = single(1); 
obstpos = single([5, 5, 5]);
obstrad = single(1);
printsphere(obstpos, obstrad)


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
            [Uatt, Fatt] = attractive(d, xi1, xi2, drones(i,:), goaldrones(i,:), rhogoal);

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
            vvec = F;
            drones(i,:) = vvec*tstep + drones(i,:);
            figure(1)
            xlabel('x / m')
            ylabel('y /m') 
            zlabel('z /m')
            printspherecolor(drones(i,:), raddrones, colordrones(i,:))

            %Printing distance to goal from the surface of the drone
            disttogoal = norm(drones(i,:)-goaldrones(i,:)-raddrones);
            figure(2)
            xlabel('time / s')
            ylabel('distance to goal / m') 
            scatter(iteration*tstep, disttogoal, 10, colordrones(i,:))
            hold on
            grid on
            
            %Printing distance to closest obstacle, dynamic or static
            closestdist = min(closestdiststat, closestdistdynam)- raddrones;
            figure(3)
            xlabel('time / s')
            ylabel('distance to closest obstacle / m') 
            scatter(iteration*tstep, closestdist, 10, colordrones(i,:))
            hold on
            grid on
            
            %Printing velocity (norm)
            vel = norm(vvec);
            figure(4)
            xlabel('time / s')
            ylabel('Norm of velocity / m/s') 
            scatter(iteration*tstep, vel, 10, colordrones(i,:))
            hold on
            grid on
            
        end
    end

    pause(0.01)
%     figure(1)
%     frame1 = getframe(gcf);
%     writeVideo(environment_video_1, frame1);
%     view([150 30])
%     frame2 = getframe(gcf);
%     writeVideo(environment_video_2, frame2);
%     view([-160 30])
%     frame3 = getframe(gcf);
%     writeVideo(environment_video_3, frame3);
    iteration = iteration + 1 ;
end

% close(environment_video_1);
% close(environment_video_2);
% close(environment_video_3);

