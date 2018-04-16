function printcirclecolor(pos, rad, color)
    hold on
    th = 0:pi/50:2*pi;
    xunit = rad * cos(th) + pos(1);
    yunit = rad * sin(th) + pos(2);
    plot(xunit, yunit, 'Color', color);
    hold on
end