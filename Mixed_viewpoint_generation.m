close all
clear all
tic

%%%%% Mesh import %%%%%
[v,f,n,~] = stlRead('big_ben_2_convex_hull.stl'); %Convex hull of the mesh, in .stl format
[v2,f2,n2,~] = stlRead('big_ben_2_16k.stl'); %Mesh (Inspection target), in .stl format
name = 'Simple Structure';
%%%%% End of Mesh import %%%%%

dimensions = zeros(2,3);
dimensions(1,:) = max(v2);
dimensions(2,:) = abs(min(v2));
max_dimensions = max(dimensions);

%%%%% Paramater Tuning %%%%%
D = 20; % Safety/ viewing distance
FOV = 42; % FOV of the camera, in deg
FOV = (FOV/180)*pi;
vert_overlap = 4; % @Overlapping rate, determine the viewpoint density
height_lower = 5; % @Min. height for the visibility, i.e. surfaces patches located below this will not be count towards the overall visibility
height_upper = 92; % @Max. height
cov_perct = 0.9; % @Coverage percentage to stop gap-filling generation
no_layer = 6; %@Determine the no. of layer for the ML-ADTSP
% @: Adjust based on the inspection target
%%%%% End of Parameter Tuning %%%%%

slicing_size = (2*D*tan(FOV))/vert_overlap;
slice_no = 1;
Slice.level(1).v(1).x = 0;
Slice.level(1).v(1).y = 0;
FOV2 = 60;

min_dim = min(v);
min_dim = round(min_dim);
max_dim = max(v);

origin = [(max_dim(1)-min_dim(1))/2 (max_dim(2)-min_dim(2))/2 0];
mesh_overlap = 1;


%%%%% Slicing (Revolving Viewpoints) %%%%%
disp('Starting Revolving Viewpoints Generation')
maximum = max(v, [],1);
minimum = min(v, [],1);
min_z = minimum(3);
slice_current = min_z + slicing_size;
while slice_current < maximum(3)
    [x, y] = meshgrid(minimum(1)*2:abs(maximum(1))/4:maximum(1)*2, minimum(2)*2:abs(maximum(2))/4:maximum(2)*2);
    z = zeros(size(x));
    Z = z + slice_current;
    j = 1;
    Activate_flag = 0;
    color = rand(1,3);
    for i = 1:size(v, 1)
        if v(i,3)< (slice_current+(slicing_size/3)) && v(i,3) > slice_current %The altitude buffer need to be adjust 
            Activate_flag = 1;
            Slice.level(slice_no).v(j).x = v(i,1);
            Slice.level(slice_no).v(j).y = v(i,2);
            Slice.level(slice_no).v(j).z = v(i,3);
            j = j+1;
        end
    end
    if Activate_flag == 0
        slice_current = slice_current + slicing_size;
        continue
    end
    for i = 1: size(Slice.level(slice_no).v,2)
        temp_x(i) = Slice.level(slice_no).v(i).x;
        temp_y(i) = Slice.level(slice_no).v(i).y;
    end
    [k,avl] = convhull(temp_x,temp_y, 'Simplify', true);
    temp_z = slice_current+zeros(1,size(temp_x(k),2));
    Slice.level(slice_no).conX = temp_x(k);
    Slice.level(slice_no).conY = temp_y(k);
    Slice.level(slice_no).conZ = temp_z;
    A = polyshape(Slice.level(slice_no).conX, Slice.level(slice_no).conY);
    polyout1 = polybuffer(A,D);
    peri(slice_no) = perimeter(polyout1);
    for i = 1:size(polyout1.Vertices,1)
        Path_base.level(slice_no).bufferX(i) = polyout1.Vertices(i,1);
        Path_base.level(slice_no).bufferY(i) = polyout1.Vertices(i,2);
    end
    Path_base.level(slice_no).bufferX(i+1) = polyout1.Vertices(1,1); %add an extra point to complete the polygon
    Path_base.level(slice_no).bufferY(i+1) = polyout1.Vertices(1,2);
    Path_base.level(slice_no).bufferZ = slice_current+zeros(1,size(polyout1.Vertices,1)+1);
    slice_current = slice_current + slicing_size;
    slice_no = slice_no +1;
end

No_section = round(max(peri)/slicing_size);

%%%%% End of Slicing %%%%%

%%%%% viewpoints Generation %%%%%
waypoint_list = [];
for i = 1:slice_no-1
    Waypoint.level(i).Division = Waypoint_Generation(Path_base, [origin(1) origin(2)], No_section, D, i).Division;
    for j = 1:size(Waypoint.level(i).Division,2)
        Waypoint.level(i).Division(j).Z = Path_base.level(i).bufferZ(1);
        waypoint_list = [waypoint_list; Waypoint.level(i).Division(j).X Waypoint.level(i).Division(j).Y Waypoint.level(i).Division(j).Z];
    end
end
% Enable this for visualization
% plot3(waypoint_list(:,1), waypoint_list(:,2), waypoint_list(:,3), 'r.','markersize', 15);

%%%%% End of viewpoints Generation %%%%%

%%%%% Discretization Setup %%%%%
grid_size = 1.1*D;
Q1_bound = grid_size-rem(max(v2),grid_size)+max(v2)+3*grid_size;
Q3_bound = min(v2)-(grid_size+rem(min(v2),grid_size))-3*grid_size;
Q3_bound(3) = 0;
no_of_grid = (Q1_bound-Q3_bound)/grid_size;
Grid_INFO = [Q1_bound(1); Q1_bound(2);Q1_bound(3);Q3_bound(1);Q3_bound(2);Q3_bound(3);grid_size];
disp('Number of grid:')
disp((Q1_bound-Q3_bound)/grid_size)

no_grid = ((Q1_bound(1)-Q3_bound(1))/grid_size)*((Q1_bound(2)-Q3_bound(2))/grid_size)*((Q1_bound(3)-Q3_bound(3))/grid_size);
ALL_grid(no_grid).point = [];
%%%%% End of Discretization Setup %%%%%


%%%%% Compute centroid location %%%%%
Centroid_list = zeros(size(f2));
for i = 1:size(f2,1)
    p1 = v2(f2(i,1),:);
    p2 = v2(f2(i,2),:);
    p3 = v2(f2(i,3),:);
    A = [p1; p2; p3];
    centroid1 = mean(A);
    Centroid_list(i,:) = centroid1;

    [gridID,~] = getGridID(centroid1, grid_size, Q3_bound(1), Q1_bound(1), Q3_bound(2), Q1_bound(2), Q3_bound(3), Q1_bound(3),3,false);
    ALL_grid(gridID).point = [ALL_grid(gridID).point; centroid1 i];
end
%%%%% End of compute centroid location %%%%%

%%%%% Discritize vertices %%%%%
all_grid_id = size(f2,1);
for i = 1:size(v2,1)
    temp_point = v2(i,:);
    [gridID,~] = getGridID(temp_point, grid_size, Q3_bound(1), Q1_bound(1), Q3_bound(2), Q1_bound(2), Q3_bound(3), Q1_bound(3),3,false);
    ALL_grid(gridID).point = [ALL_grid(gridID).point; temp_point i+all_grid_id];
end

for i = 1:size(ALL_grid,2)
    if isempty(ALL_grid(i).point)
        continue
    end
    temp_points = ALL_grid(i).point(:,1:3);
end
%%%%% End of Discritize vertices %%%%%

%%%%% Surface normal correction %%%%%
z_value = Centroid_list(:,3);
min_cen_z = min(z_value);
max_cen_z = max(z_value);
num_layer = 7;
layer_thickness = (max_cen_z-min_cen_z)/num_layer;
for i = 1:num_layer
    lower_bound = min_cen_z + (i-1)*layer_thickness;
    upper_bound = min_cen_z + i*layer_thickness;
    layer_centroids = Centroid_list(Centroid_list(:, 3) >= lower_bound & Centroid_list(:, 3) < upper_bound, :);
    % Calculate the centroid of the current layer
    if ~isempty(layer_centroids)
        layer_centroid = mean(layer_centroids, 1);
    
        % Check normals for centroids in the current layer
        for ii = 1:size(Centroid_list, 1)
            if Centroid_list(ii, 3) >= lower_bound && Centroid_list(i, 3) < upper_bound
                % Get the normal of the current patch
                temp_normal = n2(ii, :);
                temp_centroid = Centroid_list(ii, :);
                
                % Calculate the direction vector from the face centroid to the layer centroid
                direction_vector = layer_centroid - temp_centroid;
                
                % Check if the normal is pointing inward
                if dot(temp_normal, direction_vector) > 0
                    % Invert the normal if it points inward
                    n2(ii, :) = -temp_normal;
                end
            end
        end
    end
end
%%%%% End of surface normal correction %%%%%
%%%%% End of revolving viewpoint generation %%%%%

%%%%% Visibility Estimation %%%%%
% stlPlot(v2,f2,'Revolving Viewpoints');
% axis equal;
% hold on
% xlabel 'x (m)'
% ylabel 'y (m)'
% zlabel 'z (m)'
Viewing_distance = 1.25*D/cos(FOV);
Visibility(1).visibility = [];
Overall_visibility = zeros(size(Centroid_list,1),1);
viewing_direction = zeros(size(waypoint_list));
vector2surface = zeros(size(waypoint_list));
viewing_angle_threshold = 60;
for i = 1:size(waypoint_list,1)
        temp_point = waypoint_list(i,:);
        [gridID,neighbour] = getGridID(temp_point, grid_size, Q3_bound(1), Q1_bound(1), Q3_bound(2), Q1_bound(2), Q3_bound(3), Q1_bound(3),3,true);
        Part1 = ALL_grid(gridID).point;
        PointID_P1 = [];
        PointID_P2 = [];
        if ~isempty(Part1)
            PointID_P1 = ALL_grid(gridID).point(:,end);
            Part1 = Part1(:,1:3);
        end
        Part2 = [];
        for k = 1:size(neighbour,1)
            Part2 = [Part2; ALL_grid(neighbour(k)).point];
        end
    
        if ~isempty(Part2)
            PointID_P2 = Part2(:,end);
            Part2(:,end) = [];
            PointID = [PointID_P1; PointID_P2];
        else
            PointID = PointID_P1;
        end
        neighbour_centroid = [Part1; Part2];
        closest_point = [];
        current_shortest_dist = [];
        for j = 1:size(neighbour_centroid,1)
            temp_centroid = neighbour_centroid(j,:);
            temp_id = find(v2 == temp_centroid,2);
            if ~isempty(temp_id)
                continue
            end
            temp_dist = norm(temp_point-temp_centroid);
            if isempty(current_shortest_dist)
                current_shortest_dist = temp_dist;
                closest_point = temp_centroid;
            else
                if temp_dist < current_shortest_dist
                    current_shortest_dist = temp_dist;
                    closest_point = temp_centroid;
                end
            end
        end
        if isempty(closest_point)
            Visibility(i).visibility = zeros(size(Centroid_list,1),1);
            continue
        end
        vectorToSurface = temp_point - closest_point;
        vectorToSurface = vectorToSurface/norm(vectorToSurface);
        vector2surface(i,:) = vectorToSurface;
        temp_normal = n2(all(Centroid_list == closest_point,2),:);
        perpendicularDistance = abs(dot(vectorToSurface, temp_normal));
        viewing_direction(i,:) = -vectorToSurface;
        vist = [];
        for k = 1:size(neighbour_centroid,1)
            temp_centroid = neighbour_centroid(k,:);
            temp_dist = norm(temp_point-temp_centroid);
            temp_id = find(all(v2 == temp_centroid,2));
            if ~isempty(temp_id)
                continue
            end
            id = all(Centroid_list == temp_centroid,2);
            temp_normal = n2(id,:);
            if size(temp_normal,1) > 1
                temp_normal = temp_normal(1,:);
            end
            isAngleOfIncidenceLarge = detectLargeAngleOfIncidence(temp_normal, viewing_direction(i,:), viewing_angle_threshold);
            if temp_dist > Viewing_distance || isAngleOfIncidenceLarge
                continue
            else
                isWithinViewCone = CalPointVist(temp_point, viewing_direction(i,:), temp_centroid, FOV2);
                if isWithinViewCone
                    id = find(id);
                    vist = [vist; id];
                end
            end
        end
        if isempty(vist)
            Visibility(i).visibility = zeros(size(Centroid_list,1),1);
            continue
        else
            vist = unique(vist);
            temp_vist = zeros(size(Centroid_list,1),1);
            temp_vist(vist,:) = 1;
            Visibility(i).visibility = temp_vist;
            Overall_visibility = Overall_visibility + temp_vist;
        end
end

%%%%% Rearrage visibility data %%%%%
Viewpoint.v(1).x = 0;
Viewpoint.v(1).y = 0;
Viewpoint.v(1).z = 0;
for i = 1:size(waypoint_list,1)
    Viewpoint.v(i).x = waypoint_list(i,1);
    Viewpoint.v(i).y = waypoint_list(i,2);
    Viewpoint.v(i).z = waypoint_list(i,3);
    Viewpoint.v(i).visibility = Visibility(i).visibility;
end

%%%%% Identify out of range surfaces %%%%%
LowG_flag = 10*max(Overall_visibility);
flag_id = Centroid_list(:,3) < height_lower | Centroid_list(:,3) > height_upper;
Overall_visibility(flag_id) = LowG_flag;
flag_id = n2(:,3) == 1 | n2(:,3) == -1;
Overall_visibility(flag_id) = LowG_flag;
flag_id2 = zeros(size(n2,1),1);
for i = 1:size(n2,1)
    temp_normal = n2(i,:);
    pitch_angle_rad = atan2(temp_normal(3), sqrt(temp_normal(1)^2 + temp_normal(2)^2));
    pitch_angle_deg = rad2deg(pitch_angle_rad);
    if pitch_angle_deg < -60
        flag_id2(i) = 1;
        Overall_visibility(i) = LowG_flag; %Set mesh looking down as visible
    end

end
flag_id2 = logical(flag_id2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% Visiualize revolving VP w/ corresponding coverages %%%%%
% stlPlot(v2,f2,'Revolving Viewpoints');
% axis equal;
% hold on
% xlabel 'x (m)'
% ylabel 'y (m)'
% zlabel 'z (m)'
% plot3(waypoint_list(:,1), waypoint_list(:,2), waypoint_list(:,3),'r.');
% quiver3(waypoint_list(:,1), waypoint_list(:,2), waypoint_list(:,3), viewing_direction(:,1), viewing_direction(:,2), viewing_direction(:,3));
% for i = 1:size(Centroid_list,1)
%     if Overall_visibility(i) > 0
%         fill3([v2(f2(i,1),1) v2(f2(i,2),1) v2(f2(i,3),1)], [v2(f2(i,1),2) v2(f2(i,2),2) v2(f2(i,3),2)],[v2(f2(i,1),3) v2(f2(i,2),3) v2(f2(i,3),3)],[0 1 0], 'EdgeColor','none');
%     end
% end
%%%%% End of Visibility Estimation (Revolving Viewpoints) %%%%%

%%%%% Gap filling viewpoint generation %%%%%
unscanned_mesh = find(~Overall_visibility);
Dilate_dist = D;
Dilate_view_dist = Dilate_dist/cos(FOV);
Collision_dist = D;
disp('Starting Gap-filling Viewpoint Generation')
while true
    id = 1;
    unscanned_mesh_id = unscanned_mesh(id);
    surface_normal = n2(unscanned_mesh_id,:);
    [temp_vp, temp_vd, gap_fill_vist] = Gap_fill_function_v2(unscanned_mesh_id, surface_normal, Dilate_dist, D, ALL_grid, Grid_INFO, Centroid_list, n2, viewing_angle_threshold, FOV2, Dilate_view_dist, layer_thickness/2, v2); %DOUBLE CHECK THIS LINE
    temp_id = size(Viewpoint.v,2);
    Viewpoint.v(temp_id+1).x = temp_vp(1);
    Viewpoint.v(temp_id+1).y = temp_vp(2);
    Viewpoint.v(temp_id+1).z = temp_vp(3);
    Viewpoint.v(temp_id+1).visibility = gap_fill_vist;
    waypoint_list = [waypoint_list; temp_vp];
    viewing_direction = [viewing_direction; temp_vd];

    Overall_visibility = Overall_visibility + gap_fill_vist;
    unscanned_mesh = find(~Overall_visibility);
    disp('Coverage Percentage:')
    disp(size(find(Overall_visibility),1)/size(Overall_visibility,1))
    if size(find(Overall_visibility),1)/size(Overall_visibility,1) > cov_perct
        break
    end
end
uncovered_id = Overall_visibility == 0;
Overall_visibility(uncovered_id) = LowG_flag;
unseen = Overall_visibility < mesh_overlap;
unseen = find(unseen);
if ~isempty(unseen)
    for i = 1:size(unseen,1)    
        repeat = 1;
        for ii = unseen(unseen_id(i),2):mesh_overlap
            repeat = 1;
            while repeat == 1
                temp_vp = Gap_filling_function(Face, v, slicing_size, unseen_id(i), n2(unseen_id(i),:), Dilate_dist, Dilate_view_dist, Collision_dist, n2, FOV,D, v2);
                for iii = 1:size(Viewpoint.v,2)
                    if temp_vp.v(1).x == Viewpoint.v(iii).x && temp_vp.v(1).y == Viewpoint.v(iii).y && temp_vp.v(1).z == Viewpoint.v(iii).z
                        break
                    end
                    repeat = 0;
                    break
                end
            end
            temp_id = size(Viewpoint.v,2);
            Viewpoint.v(temp_id+1).x = temp_vp.v(1).x;
            Viewpoint.v(temp_id+1).y = temp_vp.v(1).y;
            Viewpoint.v(temp_id+1).z = temp_vp.v(1).z;
            Viewpoint.v(temp_id+1).visibility = temp_vp.v(1).Visibility;
            Viewpoint.v(temp_id+1).poi = [Face.v(unseen_id(i)).x Face.v(unseen_id(i)).y Face.v(unseen_id(i)).z];
            Overall_visibility = Overall_visibility + temp_vp.v(1).Visibility;
        end
    end
end
%%%%% Debug %%%%%
flag_id = Centroid_list(:,3) < height_lower | Centroid_list(:,3) > height_upper;
Overall_visibility(flag_id) = LowG_flag;
Overall_visibility(flag_id) = LowG_flag;
flag_id = n2(:,3) == 1 | n2(:,3) == -1;
Overall_visibility(flag_id) = LowG_flag;
Overall_visibility(flag_id2) = LowG_flag;

if max(Overall_visibility) ~= LowG_flag
    error = 1;
end
%%%%% End of Debug %%%%%
%%%%% End of Gap filling viewpoint generation %%%%%


%%%%% Plot all viewpoints %%%%%
stlPlot(v2,f2,'All Viewpoints');
axis tight;
hold on
xlabel 'x (m)'
ylabel 'y (m)'
zlabel 'z (m)'
plot3(waypoint_list(:,1), waypoint_list(:,2), waypoint_list(:,3), 'r.');
quiver3(waypoint_list(:,1), waypoint_list(:,2), waypoint_list(:,3), viewing_direction(:,1), viewing_direction(:,2), viewing_direction(:,3))
%%%%% End of plot %%%%%

%%%% Solve SCP Problem %%%%%
disp('Starting SCP')
[SCP_solution] = SCP_function(Overall_visibility, Viewpoint, LowG_flag, mesh_overlap);
%%%%% End of SCP %%%%%

%%%%%% Visualize SCP result %%%%%
stlPlot(v2,f2,'Resulting Viewpoints and Viewing Direction');
axis tight;
hold on
xlabel 'x (cm)'
ylabel 'y (cm)'
zlabel 'z (cm)'
plot3(waypoint_list(SCP_solution,1), waypoint_list(SCP_solution,2), waypoint_list(SCP_solution,3),'k.')
quiver3(waypoint_list(SCP_solution,1), waypoint_list(SCP_solution,2), waypoint_list(SCP_solution,3), viewing_direction(SCP_solution,1), viewing_direction(SCP_solution,2), viewing_direction(SCP_solution,3))
vp_x = zeros(size(Viewpoint.v,2),1);
vp_y = vp_x;
vp_z = vp_x;
vp_poi = zeros(size(Viewpoint.v,2), 3);
%%%%% End of visualization %%%%%

%%%%% Viewpoint collision check %%%%%
result_viewpoint = waypoint_list(SCP_solution,:);
replan_dist = D;
toc
vp_time = toc;
increment = 1;
for i = 1:size(result_viewpoint,1)
    need_replan = false;
    temp_point = result_viewpoint(i,:);
    [gridID,neighbour] = getGridID(temp_point, grid_size, Q3_bound(1), Q1_bound(1), Q3_bound(2), Q1_bound(2), Q3_bound(3), Q1_bound(3),3,true);
    if gridID > size(ALL_grid,2) || gridID < 1
        Part1 = [];
    else
    Part1 = ALL_grid(gridID).point;
    end
    PointID_P1 = [];
    PointID_P2 = [];
    if ~isempty(Part1)
        PointID_P1 = ALL_grid(gridID).point(:,end);
        Part1 = Part1(:,1:3);
    end
    Part2 = [];
    for k = 1:size(neighbour,1)
        Part2 = [Part2; ALL_grid(neighbour(k)).point];
    end

    if ~isempty(Part2)
        PointID_P2 = Part2(:,end);
        Part2(:,end) = [];
        PointID = [PointID_P1; PointID_P2];
    else
        PointID = PointID_P1;
    end
    neighbour_centroid = [Part1; Part2];

    for k = 1:size(neighbour_centroid,1)
        temp_centroid = neighbour_centroid(k,:);
        temp_dist = norm(temp_point-temp_centroid);
        if temp_dist < D
            need_replan = true;
            break
        end
    end
    if need_replan
        iteration = 1;
        group_centroid = mean(neighbour_centroid);
        escape_dir = temp_point - group_centroid;
        escape_dir = escape_dir/norm(escape_dir);
        while true
            escape_dist = iteration*increment; 
            temp_point = temp_point + escape_dist*escape_dir;
            failed = false;
            for k = 1:size(neighbour_centroid,1)
                temp_centroid = neighbour_centroid(k,:);
                temp_dist = norm(temp_point - temp_centroid);
                if temp_dist < D
                    failed = true;
                    break
                end
            end
            if failed
                iteration = iteration + 1;
            else
                break
            end
        end
        plot3(result_viewpoint(i,1), result_viewpoint(i,2), result_viewpoint(i,3),'r.','MarkerSize',15)
        result_viewpoint(i,:) = temp_point;
        plot3(temp_point(1), temp_point(2), temp_point(3),'g.','MarkerSize',15)
    end
    
end

%%%%% ADTSP Rework %%%%%
D = 0.7*D;
layer(1).waypoint = [];
layer_thickness = (max(result_viewpoint(:,3))-min(result_viewpoint(:,3)))/no_layer;
Q1_bound = [Grid_INFO(1) Grid_INFO(2) Grid_INFO(3)];
Q3_bound = [Grid_INFO(4) Grid_INFO(5) Grid_INFO(6)];
grid_size = Grid_INFO(end);
tour = [];
layer_sol(1).points = [];

%%%%% Solve ADTSP at each level %%%%%
for i = 1:no_layer
    lower_bound = min(result_viewpoint(:,3)) + (i-1)*layer_thickness;
    upper_bound = min(result_viewpoint(:,3)) + i*layer_thickness;
    layer_waypoints = result_viewpoint(result_viewpoint(:, 3) >= lower_bound & result_viewpoint(:, 3) < upper_bound, :);
    if size(layer_waypoints,1) < 3
        prev_waypoint = layer_waypoints;
        merge = true;
        continue
    else
        merge = false; 
        prev_waypoint = [];
    end
    layer_waypoints = [layer_waypoints; prev_waypoint];
    layer(i).waypoint = layer_waypoints;
    solution = GA_3D(layer_waypoints);
    uncheck_sol(i).path = solution;
    updated_solution = solution;
    for j = 1:size(solution,1)-1
        p1 = solution(j,:);
        p2 = solution(j+1,:);
        dir1 = atan2(p2(1)-p1(1), p2(2)-p1(2));
        xy_dist = norm(p2(1:2)-p1(1:2));
        dir2 = atan2(p2(3)-p1(3),xy_dist);
        path_dist = norm(p2-p1);
        need_replan = false;
        for ii = 0:path_dist
            temp_coor = p1+ii.*[sin(dir1) cos(dir1) tan(dir2)];
            [gridID,neighbour] = getGridID(temp_coor, grid_size, Q3_bound(1), Q1_bound(1), Q3_bound(2), Q1_bound(2), Q3_bound(3), Q1_bound(3),3,true);
            if gridID > size(ALL_grid,2) || gridID < 1
                Part1 = [];
            else
            Part1 = ALL_grid(gridID).point;
            end
            PointID_P1 = [];
            PointID_P2 = [];
            if ~isempty(Part1)
                PointID_P1 = ALL_grid(gridID).point(:,end);
                Part1 = Part1(:,1:3);
            end
            Part2 = [];
            for k = 1:size(neighbour,1)
                Part2 = [Part2; ALL_grid(neighbour(k)).point];
            end
            if ~isempty(Part2)
                PointID_P2 = Part2(:,end);
                Part2(:,end) = [];
                PointID = [PointID_P1; PointID_P2];
            else
                PointID = PointID_P1;
            end
            neighbour_centroid = [Part1; Part2];
            for k = 1:size(neighbour_centroid,1)
                temp_centroid = neighbour_centroid(k,:);
                temp_dist = norm(temp_coor-temp_centroid);
                if temp_dist < D
                    giga = 1;
                    need_replan = true;
                    break
                end
            end
            if need_replan
                break
            end
        end
        if need_replan
            final_smooth_path = RRT_updated(p1, p2, Centroid_list, D, [], ALL_grid, Grid_INFO, result_viewpoint);
            updated_solution = [updated_solution(1:find(all(updated_solution == p1,2)),:); final_smooth_path ; updated_solution(find(all(updated_solution == p2,2)):end,:)];
        end
    end
    layer_sol(i).points = updated_solution;
    tour = [tour; updated_solution];
end
%%%%% End of solving ADTSP at each level %%%%%

%%%%% Connect path segment at different levels %%%%%
final_solution = [];
num_steps = 50;
for i = 1:size(layer_sol,2)-1
    lower_level = layer_sol(i).points;
    higher_level = layer_sol(i+1).points;
    need_replan = false;
    clear all_comb
    %%%%% Calculate the cost for each combination %%%%%
    if i == 1
        %%%%% The first layer will have 4 possible combination %%%%%
        %%%%% combination 1 (lower_level --> higher_level)
        temp_route = [lower_level; higher_level];
        all_comb(1).sol = temp_route;
        cost = zeros(4,1);
        for k = 1:size(temp_route,1)-1
            temp_dist = norm(temp_route(k+1,:)-temp_route(k,:));
            cost(1) = cost(1) + temp_dist;
        end
        %%%%% Combination 2 (flip(lower_level --> HL))
        temp_route = [flip(lower_level); higher_level];
        all_comb(2).sol = temp_route;
        for k = 1:size(temp_route,1)-1
            temp_dist = norm(temp_route(k+1,:)-temp_route(k,:));
            cost(2) = cost(2) + temp_dist;
        end
        %%%%% Combination 3 (LL --> flip(HL))
        temp_route = [lower_level; flip(higher_level)];
        all_comb(3).sol = temp_route;
        for k = 1:size(temp_route,1)-1
            temp_dist = norm(temp_route(k+1,:)-temp_route(k,:));
            cost(3) = cost(3) + temp_dist;
        end
        %%%%% Combination 4 (flilp(LL)-->flip(HL))
        temp_route = [flip(lower_level); flip(higher_level)];
        all_comb(4).sol = temp_route;
        for k = 1:size(temp_route,1)-1
            temp_dist = norm(temp_route(k+1,:)-temp_route(k,:));
            cost(4) = cost(4) + temp_dist;
        end
        [~, id] = min(cost);
        if id == 3 || id ==4
            %%%%% Tell subsequent iteration that HL is flip %%%%%
            isPrev_flip = true;
        else
            isPrev_flip = false;
        end
        shortest_route = all_comb(id).sol;
        part1 = shortest_route(1:size(lower_level,1),:);
        part2 = shortest_route(size(lower_level,1)+1:end,:);
        p1 = part1(end,:);
        p2 = part2(1,:);
        for l = 0:num_steps
            temp_point = p1 + (p2-p1)*(l/num_steps);
            [gridID,neighbour] = getGridID(temp_point, grid_size, Q3_bound(1), Q1_bound(1), Q3_bound(2), Q1_bound(2), Q3_bound(3), Q1_bound(3),3,true);
            if gridID > size(ALL_grid,2) || gridID < 1
                Part1 = [];
            else
            Part1 = ALL_grid(gridID).point;
            end
            PointID_P1 = [];
            PointID_P2 = [];
            if ~isempty(Part1)
                PointID_P1 = ALL_grid(gridID).point(:,end);
                Part1 = Part1(:,1:3);
            end
            Part2 = [];
            for k = 1:size(neighbour,1)
                Part2 = [Part2; ALL_grid(neighbour(k)).point];
            end
            if ~isempty(Part2)
                PointID_P2 = Part2(:,end);
                Part2(:,end) = [];
                PointID = [PointID_P1; PointID_P2];
            else
                PointID = PointID_P1;
            end
            neighbour_centroid = [Part1; Part2];
            for k = 1:size(neighbour_centroid,1)
                temp_centroid = neighbour_centroid(k,:);
                temp_dist = norm(temp_point-temp_centroid);
                if temp_dist < D
                    giga = 1;
                    need_replan = true;
                    break
                end
            end
            if need_replan
                break
            end
        end
        if need_replan
            temp_path = RRT_updated(p1, p2, Centroid_list, D, [], ALL_grid, Grid_INFO, result_viewpoint);
            temp_path = [part1; temp_path; part2];
            final_solution = [final_solution; temp_path];
        else
            final_solution = [final_solution; all_comb(id).sol];
        end
    else
        %%%%% The rest will only have 2 possible combinations %%%%%
        if isPrev_flip
            lower_level = flip(lower_level);
        else
            % lower_level = layer_sol(i).sol;
        end
        cost = zeros(2,1);
        %%%%% Combination 1 LL --> HL
        temp_route = [lower_level; higher_level];
        all_comb(1).sol = higher_level;
        for k = 1:size(temp_route,1)-1
            temp_dist = norm(temp_route(k+1,:)-temp_route(k,:));
            cost(1) = cost(1) + temp_dist;
        end
        %%%%% Combination 2 LL --> flip(HL)
        temp_route = [lower_level; flip(higher_level)];
        all_comb(2).sol = flip(higher_level);
        for k = 1:size(temp_route,1)-1
            temp_dist = norm(temp_route(k+1,:)-temp_route(k,:));
            cost(2) = cost(2) + temp_dist;
        end
        [~, id] = min(cost);
        if id == 2
            %%%%% Tell subsequent iteration that HL is flip %%%%%
            isPrev_flip = true;
        else
            isPrev_flip = false;
        end
        shortest_route = all_comb(id).sol;
        part1 = lower_level;
        part2 = shortest_route;
        p1 = part1(end,:);
        p2 = part2(1,:);
        for l = 0:num_steps
            temp_point = p1 + (p2-p1)*(l/num_steps);
            [gridID,neighbour] = getGridID(temp_point, grid_size, Q3_bound(1), Q1_bound(1), Q3_bound(2), Q1_bound(2), Q3_bound(3), Q1_bound(3),3,true);
            if gridID > size(ALL_grid,2) || gridID < 1
                Part1 = [];
            else
            Part1 = ALL_grid(gridID).point;
            end
            PointID_P1 = [];
            PointID_P2 = [];
            if ~isempty(Part1)
                PointID_P1 = ALL_grid(gridID).point(:,end);
                Part1 = Part1(:,1:3);
            end
            Part2 = [];
            for k = 1:size(neighbour,1)
                Part2 = [Part2; ALL_grid(neighbour(k)).point];
            end
            if ~isempty(Part2)
                PointID_P2 = Part2(:,end);
                Part2(:,end) = [];
                PointID = [PointID_P1; PointID_P2];
            else
                PointID = PointID_P1;
            end
            neighbour_centroid = [Part1; Part2];
            for k = 1:size(neighbour_centroid,1)
                temp_centroid = neighbour_centroid(k,:);
                temp_dist = norm(temp_point-temp_centroid);
                if temp_dist < D
                    giga = 1;
                    need_replan = true;
                    break
                end
            end
            if need_replan
                break
            end
        end
        if need_replan
            temp_path = RRT_updated(p1, p2, Centroid_list, D, [], ALL_grid, Grid_INFO, result_viewpoint);
            temp_path = [temp_path; part2];
            final_solution = [final_solution; temp_path];
        else
            final_solution = [final_solution; all_comb(id).sol];
        end
    end
end
highest_point = result_viewpoint(find(~ismember(result_viewpoint, final_solution,'rows')),:);
if ~isempty(highest_point)
fin_sol2 = final_solution;
p1 = final_solution(end,:);
p2 = highest_point(1,:); 
for l = 1:size(highest_point,1)
    if l >= 2
        p1 = highest_point(l-1,:); 
    end
    p2 = highest_point(l,:); 
    for i = 0:num_steps
        temp_point = p1 + (p2-p1)*(i/num_steps);
        [gridID,neighbour] = getGridID(temp_point, grid_size, Q3_bound(1), Q1_bound(1), Q3_bound(2), Q1_bound(2), Q3_bound(3), Q1_bound(3),3,true);
        if gridID > size(ALL_grid,2) || gridID < 1
            Part1 = [];
        else
        Part1 = ALL_grid(gridID).point;
        end
        PointID_P1 = [];
        PointID_P2 = [];
        if ~isempty(Part1)
            PointID_P1 = ALL_grid(gridID).point(:,end);
            Part1 = Part1(:,1:3);
        end
        Part2 = [];
        for k = 1:size(neighbour,1)
            Part2 = [Part2; ALL_grid(neighbour(k)).point];
        end
        if ~isempty(Part2)
            PointID_P2 = Part2(:,end);
            Part2(:,end) = [];
            PointID = [PointID_P1; PointID_P2];
        else
            PointID = PointID_P1;
        end
        neighbour_centroid = [Part1; Part2];
        for k = 1:size(neighbour_centroid,1)
            temp_centroid = neighbour_centroid(k,:);
            temp_dist = norm(temp_point-temp_centroid);
            if temp_dist < D
                giga = 1;
                need_replan = true;
                break
            end
        end
        if need_replan
            break
        end
    end
    if need_replan
        final_smooth_path = RRT_updated(p1, p2, Centroid_list, D, [], ALL_grid, Grid_INFO, result_viewpoint);
        fin_sol2 = [fin_sol2; final_smooth_path; p2];
    else
        fin_sol2 = [fin_sol2; p2];
    end
end
else
    fin_sol2 = final_solution;
end

stlPlot(v2,f2,'Inspection Path');
hold on
grid on
axis equal
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
plot3(waypoint_list(SCP_solution,1), waypoint_list(SCP_solution,2), waypoint_list(SCP_solution,3),'r.','MarkerSize',15)
quiver3(waypoint_list(SCP_solution,1), waypoint_list(SCP_solution,2), waypoint_list(SCP_solution,3), viewing_direction(SCP_solution,1), viewing_direction(SCP_solution,2), viewing_direction(SCP_solution,3))
drawnow
plot3(fin_sol2(:,1), fin_sol2(:,2), fin_sol2(:,3),'LineWidth',2,'Color','black')
%%%%% End of connection module %%%%%

overall_angle = 0;
overall_rad = 0;
for i = 1:size(fin_sol2,1)-2
    current_pt = fin_sol2(i,:);
    pt2 = fin_sol2(i+1,:);
    pt3 = fin_sol2(i+2,:);
    vector1 = pt2(1:2)-current_pt(1:2);
    vector2 = pt3(1:2)-pt2(1:2);
    dot_product = dot(vector1, vector2);
    vec1_mag = norm(vector1);
    vec2_mag = norm(vector2);
    angle_rad = acos(dot_product/(vec1_mag*vec2_mag));
    angle_deg = rad2deg(angle_rad);
    if angle_deg > 360 || isnan(angle_deg)
        continue
    end
    overall_angle = overall_angle + abs(angle_deg);
    overall_rad = overall_rad + abs(angle_rad);
end
disp("Overall Turning Angle:")
disp(overall_angle)
disp("Overall Turning Angle (rad):")
disp(overall_rad)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%