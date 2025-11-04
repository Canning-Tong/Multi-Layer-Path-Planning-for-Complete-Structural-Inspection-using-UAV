function smooth_path = RRT_updated(start, goal, Centroid_list, D, max_iterations, ALL_grid, grid_info, waypoint_list)
    % RRT parameters

    Delta = 1; % Step size % Boc: 1 % Big_Ben: 0.5
    path = [];   % Initialize path
    Q1_bound = [grid_info(1) grid_info(2) grid_info(3)];
    Q3_bound = [grid_info(4) grid_info(5) grid_info(6)];
    % bounds = [Q1_bound; Q3_bound];
    max_cen = max(Centroid_list);
    min_cen = min(Centroid_list);
    max_wp = max(waypoint_list);
    min_wp = min(waypoint_list);
    bounds = [max_cen; min_cen; max_wp; min_wp];
    grid_size = grid_info(end);
    max_grid = size(ALL_grid,2);

    % Initialize the tree
    tree(1) = struct('point', start, 'parent', 0);

    % for iter = 1:max_iterations
    while true
        % Generate a random point
        rand_point = generateRandomPoint(bounds, start, goal, D);

        % Find the nearest point in the tree
        nearest_index = nearestNeighbor(tree, rand_point);
        nearest_point = tree(nearest_index).point;

        % Steer towards the random point
        new_point = steer(nearest_point, rand_point, Delta);

        %%%% Check if new_point is inside of model %%%%%
        temp_centroid_list = Centroid_list(Centroid_list(:,3) > new_point(3) - 0.5*D & Centroid_list(:,3) < new_point(3) + 0.5*D,:);
        outline = boundary(temp_centroid_list(:,1:2));
        outline = temp_centroid_list(outline,1:2);
        if inpolygon(new_point(1), new_point(2), outline(:,1), outline(:,2))
            continue
        end

        % Check for collisions
        if ~checkCollision(nearest_point, new_point, Centroid_list, D, grid_size, Q1_bound, Q3_bound, ALL_grid)
            % Add the new point to the tree
            new_index = length(tree) + 1;
            tree(new_index) = struct('point', new_point, 'parent', nearest_index);

            % Check if the goal is reached
            if norm(new_point - goal) < 0.5*D
                path = reconstructPath(tree, new_index);

                smooth_path = path(1,:);
                j = 1;
                list = zeros(2,1);
                start_reached = 0;
                current_id = 1;
                while start_reached == 0
                    k = 0;
                    for ii = current_id:size(path,1)
                        x_current = path(current_id,:);
                        x_near = path(ii,:);
                        collision_check = false;
                        xcollision = linspace(x_near(1),x_current(1));
                        ycollision = linspace(x_near(2),x_current(2));
                        zcollision = linspace(x_near(3),x_current(3));
                        for iii = 1:100
                            temp_coor = [xcollision(iii) ycollision(iii) zcollision(iii)];
            
                            collision_check = false;
            
                            [gridID,neighbour] = getGridID(temp_coor, grid_size, Q3_bound(1), Q1_bound(1), Q3_bound(2), Q1_bound(2), Q3_bound(3), Q1_bound(3),3,true);
                            if any(neighbour == 0)
                                giga = 1;
                            end
                            neighbour(neighbour == 0) = [];
                            neighbour(neighbour > max_grid) = [];
                            if gridID < 1 || gridID > size(ALL_grid,2)
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
                            for l = 1:size(neighbour,1)
                                Part2 = [Part2; ALL_grid(neighbour(l)).point];
                            end
                        
                            if ~isempty(Part2)
                                PointID_P2 = Part2(:,end);
                                Part2(:,end) = [];
                                PointID = [PointID_P1; PointID_P2];
                            else
                                PointID = PointID_P1;
                            end
                            neighbour_centroid = [Part1; Part2];
                
                            for l = 1:size(neighbour_centroid,1)
                                temp_centroid = neighbour_centroid(l,:);
                                temp_dist = norm(temp_coor-temp_centroid);
                                if temp_dist < D
                                    giga = 1;
                                    collision_check = true;
                                    break
                                end
                            end
            
                            if collision_check
                                break
                            end
                        end
                        if collision_check
                            continue
                        end
                        k = k+1;
                        list(k) = ii;
                    end
                    [id, ~] = max(list);
                    if id == size(path,1)
                        j = j+1;
                        smooth_path(j,:) = path(id,:);
            
                        start_reached = 1;
                        continue
                    end
                    if id == current_id
                        fail_flag = 1;
                        break
                    end
                    j = j+1;
                    smooth_path(j,:) = path(id,:);
                    current_id = id;
                end


                return; % Path found
            end
        end
    end

    disp('No path found!');
end

function rand_point = generateRandomPoint(bounds, start, goal, D)
    % Calculate the bounding box of the obstacles
    min_bounds = min(bounds);
    max_bounds = max(bounds);
    max_z = max([start; goal]);
    max_z = max_z(3) + 2*D;
    min_z = min([start; goal]);
    min_z = min_z(3) - 2*D;
    if min_z < 0
        min_z = 0;
    end
    max_bounds(3) = max_z;
    min_bounds(3) = min_z;

    % Generate a random point within the bounding box
    rand_point = min_bounds + (max_bounds - min_bounds) .* rand(1, 3);
end

function nearest_index = nearestNeighbor(tree, point)
    % Find the nearest point in the tree to the given point
    distances = arrayfun(@(node) norm(node.point - point), tree);
    [~, nearest_index] = min(distances);
end

function new_point = steer(nearest_point, rand_point, Delta)
    % Move towards the random point from the nearest point
    direction = rand_point - nearest_point;
    distance = norm(direction);
    if distance > Delta
        direction = (direction / distance) * Delta; % Scale to step size
    end
    new_point = nearest_point + direction;
end

function collision = checkCollision(start_point, end_point, Centroid_list, D, grid_size, Q1_bound, Q3_bound, ALL_grid)
    % Check for collisions along the path from start_point to end_point
    num_steps = 10;
    for i = 0:num_steps
        interp_point = start_point + (end_point - start_point) * (i / num_steps);

        [gridID,neighbour] = getGridID(interp_point, grid_size, Q3_bound(1), Q1_bound(1), Q3_bound(2), Q1_bound(2), Q3_bound(3), Q1_bound(3),3,true);
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
            temp_dist = norm(interp_point-temp_centroid);
            if temp_dist < D
                collision = true;
                return;
            end
        end
    end
    collision = false; % No collision
end

function path = reconstructPath(tree, index)
    % Reconstruct the path from the tree
    path = [];
    while index > 0
        path = [tree(index).point; path]; % Add point to path
        index = tree(index).parent; % Move to parent
    end
end