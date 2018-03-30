function [totaldrones, inidrones, goaldrones, colordrones] = uavgenerator (min, max)
    %Generate UAVs
    totaldrones = randi([min,max],1,1);
    inidrones = zeros(totaldrones,3);
    goaldrones = zeros(totaldrones,3);
    colordrones = zeros(totaldrones,3);
    for i=1:totaldrones
         inidrones(i,:)= randi([0,1],1,3);
         goaldrones(i,:)= randi([9,10],1,3);
         colordrones(i,1) = rand;
         colordrones(i,2) = rand;
         colordrones(i,3) = rand;
    end
    
    %Print initial positions and goals
    for i=1:totaldrones
        scatter3(inidrones(i,1),inidrones(i,2),inidrones(i,3),3,colordrones(i,:))
        hold on
        scatter3(goaldrones(i,1),goaldrones(i,2),goaldrones(i,3),3,colordrones(i,:))
        hold on

    end
end