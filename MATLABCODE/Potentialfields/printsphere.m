function printsphere(pos, rad)
    [x,y,z] = sphere;
    surf(x*rad+pos(1), y*rad+pos(2), z*rad+pos(3));
    axis equal
    hold on
end