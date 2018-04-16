function printcircle(pos, rad)
    hold on
    d = rad*2;
    px = pos(1)-rad;
    py = pos(2)-rad;
    rectangle('Position',[px py d d],'Curvature',[1,1], 'FaceColor', [.5,.5,.5]);
    daspect([1,1,1])
    hold on
end