function [totalobst, obstpos, obstrad] = obstaclegenerator3d(min, max, totaldrones, drones, goaldrones)
    %Generate obstacles
    totalobst = randi([min,max],1,1);
    obstpos = zeros(totalobst,3);
    obstrad = zeros(totalobst,1);
    for i=1:totalobst
        %Obstacle center position: integer between 5 and 25 for X, Y and Z
        obstpos(i,:) = randi([5,25],1,3);
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
        printsphere(obstpos(i,:), obstrad(i,1))
    end
end