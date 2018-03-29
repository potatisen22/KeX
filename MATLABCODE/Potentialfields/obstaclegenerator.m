function [totalobst,obstpos] = obstaclegenerator(min,max)
    %Generate obstacles
    totalobst = randi([min,max],1,1);
    obstpos = zeros(totalobst,3);
    for i=1:totalobst
         obstpos(i,:)= randi([0,10],1,3);
    end
    
    %Print obstacles
    for i=1:totalobst
        scatter3(obstpos(i,1),obstpos(i,2),obstpos(i,3), 20,'*r')
        hold on
    end
end