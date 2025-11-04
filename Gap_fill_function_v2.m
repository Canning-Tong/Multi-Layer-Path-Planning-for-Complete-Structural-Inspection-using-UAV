function [gap_filling_vp, viewing_dir, visibility] = Gap_fill_function_v2(unscanned_mesh_id, surface_normal, dilate_dist, D, ALL_grid, grid_info, Centroid_list, n2, viewing_angle_threshold, FOV2, viewing_distance, layer_thickness, v2)

Q1_bound = [grid_info(1) grid_info(2) grid_info(3)];
Q3_bound = [grid_info(4) grid_info(5) grid_info(6)];
grid_size = grid_info(end);
interval = 1;
iteration = 0;
visibility = zeros(size(Centroid_list,1),1);
layer_thickness = layer_thickness/2;

while true
    temp_centroid = Centroid_list(unscanned_mesh_id,:);
    if iteration == 0 
        temp_normal = surface_normal;
    else
        temp_normal = surface_normal;
        dimension = randi([1 3]);
        temp_normal(dimension) = rand*temp_normal(dimension);
        if randi([0 1]) > 0
            temp_normal = -temp_normal;
        end
    end
    extended_dist = dilate_dist+floor(iteration/5);
    temp_point = temp_centroid + extended_dist*temp_normal;
    viewing_dir = -temp_normal;

    %%%%% Detect if within mesh %%%%%
    lower_bound = temp_point(3) - layer_thickness;
    if lower_bound < 0
        lower_bound = 0;
    end
    upper_bound = temp_point(3) + layer_thickness;
    layer = Centroid_list(:,3) > lower_bound & Centroid_list(:,3) < upper_bound;
    layer_list= Centroid_list(layer,:);
    outline = boundary(layer_list(:,1), layer_list(:,2));
    outline_x = layer_list(outline,1);
    outline_y = layer_list(outline,2);
    inside_flag = false;
    if inpolygon(temp_point(1), temp_point(2), outline_x, outline_y)
        iteration = iteration + 1;
        continue
    end
    [gridID,neighbour] = getGridID(temp_centroid, grid_size, Q3_bound(1), Q1_bound(1), Q3_bound(2), Q1_bound(2), Q3_bound(3), Q1_bound(3),3,true);
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

    [vp_gridID,vp_neighbor] = getGridID(temp_point, grid_size, Q3_bound(1), Q1_bound(1), Q3_bound(2), Q1_bound(2), Q3_bound(3), Q1_bound(3),3,true);
    Part1 = ALL_grid(vp_gridID).point;
    PointID_P1 = [];
    PointID_P2 = [];
    if ~isempty(Part1)
        PointID_P1 = ALL_grid(vp_gridID).point(:,end);
        Part1 = Part1(:,1:3);
    end
    Part2 = [];
    for k = 1:size(vp_neighbor,1)
        Part2 = [Part2; ALL_grid(vp_neighbor(k)).point];
    end

    if ~isempty(Part2)
        PointID_P2 = Part2(:,end);
        Part2(:,end) = [];
        PointID = [PointID_P1; PointID_P2];
    else
        PointID = PointID_P1;
    end
    vp_neighbor_centroid = [Part1; Part2];

    neighbour_centroid = [neighbour_centroid; vp_neighbor_centroid];
    neighbour_centroid = unique(neighbour_centroid,'rows');

    vist = [];
    for k = 1:size(neighbour_centroid,1)
        temp_centroid = neighbour_centroid(k,:);
        temp_id = find(all(v2 == temp_centroid,2), 1);
        if ~isempty(temp_id)
            continue
        end
        temp_dist = norm(temp_point-temp_centroid);
        if temp_dist < D
            vist = [];
            break
        end
        id = all(Centroid_list == temp_centroid,2);
        id2 = find(id);
        if id2 == unscanned_mesh_id
            giga = 1;
        end
        temp_normal = n2(id,:);
        if size(temp_normal,1) > 1
            temp_normal = temp_normal(1,:);
        end
        isAngleOfIncidenceLarge = detectLargeAngleOfIncidence(temp_normal, viewing_dir, viewing_angle_threshold);
        if temp_dist > extended_dist || isAngleOfIncidenceLarge
            continue
        else
            isWithinViewCone = CalPointVist(temp_point, viewing_dir, temp_centroid, FOV2);
            if isWithinViewCone
                id = find(id);
                vist = [vist; id];
            end
        end
    end
    if isempty(vist)
        iteration = iteration + 1;
        continue
    else
        if inside_flag
            iteration = iteration + 1;
            continue
        end
        vist = unique(vist);
        temp_vist = zeros(size(Centroid_list,1),1);
        temp_vist(vist,:) = 1;
        visibility = temp_vist;
    end
    if visibility(unscanned_mesh_id) > 0
        gap_filling_vp = temp_point;
        break
    end
    iteration = iteration + 1;
end