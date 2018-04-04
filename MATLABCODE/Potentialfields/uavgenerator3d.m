function [totaldrones, inidrones, goaldrones, colordrones] = uavgenerator3d (min, max, raddrones)
    %Generate UAVs
    totaldrones = randi([min,max],1,1);
    inidrones = zeros(totaldrones,3);
    goaldrones = zeros(totaldrones,3);
    colordrones = zeros(totaldrones,3);
    for i=1:totaldrones
         inidrones(i,:) = rand(1,3) * 30;
         goaldrones(i,:) = rand(1,3) * 30;
         colordrones(i,1) = rand;
         colordrones(i,2) = rand;
         colordrones(i,3) = rand;
    end
    
    %Print initial positions and goals
    for i=1:totaldrones
        printspherecolor(inidrones(i,:), raddrones, colordrones(i,:))
        scatter3(goaldrones(i,1),goaldrones(i,2),goaldrones(i,3),3,colordrones(i,:))
        hold on

    end
end