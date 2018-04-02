function [totalobst, obstpos, obstrad] = obstaclegenerator3d(min, max, totaldrones, drones, goaldrones)
    %Generate obstacles
    totalobst = randi([min,max],1,1);
    obstpos = zeros(totalobst,3);
    obstrad = zeros(totalobst,1);
    for i=1:totalobst
        %Obstacle center position: integer between 10 and 40 for X, Y and Z
        obstpos(i,:) = randi([10,40],1,3);
        %Obstacle radius between 0.5 and 2
        obstrad(i,1) = rand * 1.5 + 0.5; 
        
        %Obstacles can't be close to the initial position of the drones or
        %the goals
        %for j=1:totaldrones
        %   while(norm(obstpos(i,:)-))
        %end
    end
    
    %Print obstacles
    for i=1:totalobst
        %scatter3(obstpos(i,1),obstpos(i,2),obstpos(i,3), 20,'*r')
        [x,y,z] = sphere;
        surf(x*obstrad(i)+obstpos(i,1), y*obstrad(i)+obstpos(i,2), z*obstrad(i)+obstpos(i,2));
        axis equal
        hold on
    end
end