function printspherecolor(pos, rad, color)
    [x,y,z] = sphere;
    surf(x*rad+pos(1), y*rad+pos(2), z*rad+pos(3), 'FaceColor', color(:), 'EdgeColor','none');
    axis equal
    hold on
end