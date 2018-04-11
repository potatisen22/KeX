function [totalobst, obstpos, obstrad] = obstaclegenerator3d(min, max, totaldrones, drones, goaldrones, raddrones)
    %Generate obstacles
    totalobst = single(randi([min,max],1,1));
    obstpos = single(zeros(totalobst,3));
    obstrad = single(zeros(totalobst,1));
    for i=1:totalobst
        %Obstacle center position: integer between 5 and 25 for X, Y and Z
        obstpos(i,:) = single(randi([5,25],1,3));
        %Obstacle radius between 0.3 and 2
        obstrad(i,1) = single(rand * 1.7 + 0.3); 
    end
    
    %Obstacles can't be close to the initial position of the drones or
    %the goals
    %COULD WE RUN OUT OF OBSTACLES? -> FIX
    i = 1;
    while(i <= totalobst)
        j=1;
        deleted = 0;
        while(j<=totaldrones && ~deleted)
           if ((norm(obstpos(i,:) - drones(j,:)) < (raddrones + 0.3 + obstrad(i,1))) || (norm(obstpos(i,:) - goaldrones(j,:)) < (raddrones + 0.3 + obstrad(i,1)))) 
               %0.3 safe margin
               %The obstacle is deleted because it is too close to a spawn
               %or a goal
               deleted = 1;
               obstpos = single(obstpos([1:i-1,i+1:totalobst],:));
               totalobst = totalobst - 1;
           end
           j = j + 1;
        end
        if(~deleted)
            i = i + 1;
        end
    end
    
    %Print obstacles
    for i=1:totalobst
        printsphere(obstpos(i,:), obstrad(i,1))
    end
end