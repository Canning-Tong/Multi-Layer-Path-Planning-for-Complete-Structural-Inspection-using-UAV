function [Waypoint] = Waypoint_Generation(Path_base, origin, No_section, D, level_no)

    theta = (2*pi)/No_section;
    current_theta = 0;
    division = 1;
    Waypoint.Division(division).X(1) = 0;
    Waypoint.Division(division).Y(1)= 0;
    while current_theta < (2*pi)
        extend = origin + (10*D).*[sin(current_theta) cos(current_theta)];
        Line_x = [origin(1) extend(1)];
        Line_y = [origin(2) extend(2)];
        [xi, yi] = polyxpoly(Line_x, Line_y, Path_base.level(level_no).bufferX, Path_base.level(level_no).bufferY);
        for i = 1:size(xi,1)
            Waypoint.Division(division).X(i) = xi(i);
            Waypoint.Division(division).Y(i) = yi(i);
        end
        current_theta = current_theta + theta;
        division = division + 1;
    end

end