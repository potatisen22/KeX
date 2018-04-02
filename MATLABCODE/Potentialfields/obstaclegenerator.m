function [totalobst, obstpos] = obstaclegenerator(min, max, totaldrones, drones, goaldrones)
    %Generate obstacles
    totalobst = randi([min,max],1,1);
    obstpos = zeros(totalobst,3);
    for i=1:totalobst
        %Obstacle center position: integer between 10 and 40 for X, Y and Z
        obstpos(i,:) = randi([1,9],1,3);
        %Obstacle radius between 0.5 and 2 
        
        %Obstacles can't be close to the initial position of the drones or
        %the goals
        %for j=1:totaldrones
        %   while(norm(obstpos(i,:)-))
        %end
    end
    
    %Print obstacles
    for i=1:totalobst
        scatter3(obstpos(i,1),obstpos(i,2),obstpos(i,3), 20,'*r')
        hold on
    end
end