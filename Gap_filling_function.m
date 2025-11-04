%Gap filling function%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Gap_filling_VP] = Gap_filling_function(Centroid_list, v, slicing_size, surface_patch, mesh_n,Dilate_dist, Viewing_dist, Collision_dist, n, FOV, max_height)

%Initialize%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load("cameraParams.mat");
rand_min = -1;
rand_max = 1;
mesh_error_flag = 0;
problem_solved = 0;
iteration = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Camera%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mesh_locate = [Face.v(surface_patch).x Face.v(surface_patch).y Face.v(surface_patch).z];
mesh_locate = Centroid_list;
camera_coordinate = mesh_locate + Dilate_dist*mesh_n;
while problem_solved == 0
    if Point_collisin_checking_function(v, camera_coordinate, slicing_size, Collision_dist) %If collision happen due to mesh normal problem
        camera_coordinate = mesh_locate - Dilate_dist*mesh_n;
        if Point_collisin_checking_function(v, camera_coordinate, slicing_size, Collision_dist)
            mesh_error_flag = 1;
        end
    end
    if camera_coordinate(3) < 0 || mesh_error_flag == 1 || problem_solved == 0
        error_flag = 1;
        while error_flag == 1
            if iteration > 1000 %which mean the dilate distance cannot fulfill the collision constraint --> enlarge the dilation
                if logical(mod(iteration,2))
                    direction = 1;
                else
                    direction = -1;
                end
                dilate_factor = floor((iteration-1000)/1000) + 1;
                Dilate_dist = Dilate_dist + dilate_factor*0.01*Dilate_dist;
                Viewing_dist = Dilate_dist/cos(FOV*(dilate_factor*0.1));
                camera_coordinate = mesh_locate + (direction*Dilate_dist*(rand_min + (rand_max-rand_min)*rand(1,1)))*(mesh_n + [rand_min + (rand_max-rand_min)*rand(1,1) rand_min + (rand_max-rand_min)*rand(1,1) 0]);
                collision_check = Point_collisin_checking_function(v, camera_coordinate, slicing_size, Collision_dist); % 1 = collision; 0 = collision free
                [inAnglexy, inAngleyz] = angle_estimation(camera_coordinate, [mesh_locate(1) mesh_locate(2) mesh_locate(3)], mesh_n);
                if inAnglexy && inAngleyz && (~collision_check) && camera_coordinate(3) > 0 %&& camera_coordinate(3)<max_height
                    error_flag = 0;
                end
            elseif 1000>= iteration && iteration > 200 %For 200 to 1000 iteration randomness to dilate distance and mesh_n
                if logical(mod(iteration,2))
                    direction = 1;
                else
                    direction = -1;
                end
                camera_coordinate = mesh_locate + (direction*Dilate_dist*(rand_min + (rand_max-rand_min)*rand(1,1)))*(mesh_n + [rand_min + (rand_max-rand_min)*rand(1,1) rand_min + (rand_max-rand_min)*rand(1,1) 0]);
                collision_check = Point_collisin_checking_function(v, camera_coordinate, slicing_size, Collision_dist); % 1 = collision; 0 = collision free
                [inAnglexy, inAngleyz] = angle_estimation(camera_coordinate, [mesh_locate(1) mesh_locate(2) mesh_locate(3)], mesh_n);
                if inAnglexy && inAngleyz && (~collision_check) && camera_coordinate(3) > 0 %&& camera_coordinate(3)<max_height
                    error_flag = 0;
                end
            elseif iteration <= 200
                if logical(mod(iteration,2))
                    direction = 1;
                else
                    direction = -1;
                end
                camera_coordinate = mesh_locate + (direction*Dilate_dist*(rand_min + (rand_max-rand_min)*rand(1,1)))*mesh_n; %For first 200 iteration, randomness to dilate distance
                collision_check = Point_collisin_checking_function(v, camera_coordinate, slicing_size, Collision_dist); % 1 = collision; 0 = collision free
                [inAnglexy, inAngleyz] = angle_estimation(camera_coordinate, [mesh_locate(1) mesh_locate(2) mesh_locate(3)], mesh_n);
                if inAnglexy && inAngleyz && (~collision_check) && camera_coordinate(3) > 0 %&& camera_coordinate(3)<max_height
                    error_flag = 0;
                end
            end
            iteration = iteration + 1;
        end
    end
    
    cam_mesh_dist = norm(camera_coordinate-mesh_locate);
    camera_orientation = mesh_n*(-1);
    Gap_filling_VP.v(1).x = camera_coordinate(1);
    Gap_filling_VP.v(1).y = camera_coordinate(2);
    Gap_filling_VP.v(1).z = camera_coordinate(3);
    Gap_filling_VP.v(1).n = camera_orientation;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Rotate the world about z axis%%%%%%%%%%%%%%%%%%%%%%%
    dir = atan2((mesh_locate(1)-camera_coordinate(1)),(mesh_locate(2)-camera_coordinate(2))); %Rotate angle is between mesh and camera, not camera and origin
    tangle = dir;
    tRotate = axang2tform([0 0 1 (tangle)]);
    mesh_locate2 = tRotate*[mesh_locate(1); mesh_locate(2); mesh_locate(3); 1];
    rFacez.v(1).x = 0;
    rFacez.v(1).y = 0;
    rFacez.v(1).z = 0;
    for i = 1:size(Face.v,2)
        temptri = tRotate*[Face.v(i).x; Face.v(i).y; Face.v(i).z;1];
        rFacez.v(i).x = temptri(1);
        rFacez.v(i).y = temptri(2);
        rFacez.v(i).z = temptri(3);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Rotate the rectangle about x axis%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    dir2 = pi/2 - atan2((norm([camera_coordinate(1) camera_coordinate(2)]-[mesh_locate(1) mesh_locate(2)])-0),(camera_coordinate(3)-mesh_locate(3)));
    tRotate2 = axang2tform([1 0 0 (dir2)]);
    mesh_locate3 = tRotate2*[mesh_locate2(1); mesh_locate2(2); mesh_locate2(3); 1];
    rFacexz.v(1).x = 0;
    rFacexz.v(1).y = 0;
    rFacexz.v(1).z = 0;
    for i = 1:size(Face.v,2)
        temptri = tRotate2*[rFacez.v(i).x; rFacez.v(i).y; rFacez.v(i).z;1];
        rFacexz.v(i).x = temptri(1) - mesh_locate3(1);
        rFacexz.v(i).y = temptri(2);
        rFacexz.v(i).z = temptri(3) - mesh_locate3(3);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Visibility Estimationm, rotate about z axis%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    R = axang2tform([0 0 1 0]);
    R(:,4) = [];
    R(4,:) = [];
    t = [-0, 0, -(abs(mesh_locate3(2))+abs(cam_mesh_dist))];
    cm = cameraMatrix(cameraParams.Intrinsics, R,t);
    
    pixelcox = zeros(size(Face.v,2),1);
    pixelcoy = pixelcox;
    for i = 1:size(Face.v,2)
        temp_coor = [rFacexz.v(i).x rFacexz.v(i).z -rFacexz.v(i).y 1]*cm;
        pixelcox(i) = temp_coor(1)/temp_coor(3);
        pixelcoy(i) = temp_coor(2)/temp_coor(3);
    end
    visibility = zeros(size(Face.v,2),1);
    for i = 1:size(Face.v,2)
        temp_dist = norm([camera_coordinate(1) camera_coordinate(2) camera_coordinate(3)] - [Face.v(i).x Face.v(i).y Face.v(i).z]);
        [inAnglexy, inAngleyz] = angle_estimation(camera_coordinate, [Face.v(i).x Face.v(i).y Face.v(i).z], n(i,:));
        if 0<pixelcox(i) && pixelcox(i) <5472 && 0<pixelcoy(i) && pixelcoy(i)<3648 && temp_dist < Viewing_dist && inAnglexy && inAngleyz
            visibility(i) = 1;
        end
    end
    if visibility(surface_patch) == 1
        problem_solved = 1;
    end
end
Gap_filling_VP.v(1).Visibility = visibility;