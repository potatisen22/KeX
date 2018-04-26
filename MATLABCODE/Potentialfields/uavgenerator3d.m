function [totaldrones, inidrones, goaldrones, colordrones] = uavgenerator3d (min, max, raddrones)
    %Generate UAVs
    totaldrones = single(randi([min,max],1,1));
    inidrones = single(zeros(totaldrones,3));
    goaldrones = single(zeros(totaldrones,3));
    colordrones = single(zeros(totaldrones,3));
    for i=1:totaldrones
         inidrones(i,:) = single(rand(1,3) * 30);
         goaldrones(i,:) =  single(rand(1,3) * 30);
         colordrones(i,1) = single(rand);
         colordrones(i,2) = single(rand);
         colordrones(i,3) = single(rand);
    end
    
    %Print initial positions and goals
    for i=1:totaldrones
        printspherecolor(inidrones(i,:), raddrones, colordrones(i,:))
        scatter3(goaldrones(i,1),goaldrones(i,2),goaldrones(i,3),60, colordrones(i,:), 'filled', 'd')
        hold on

    end
end