%GA 3D
%Genetic Algorithm Function
%Input: num_point, point(x,y), phantom_node

function [solution_point] = GA_3D(point)
phantom_node = [100000*mean(point(:,1)) 100000*mean(point(:,2)) mean(point(:,3))];
point(size(point,1)+1,:) = phantom_node;
num_point = size(point,1);

%Initilization
No_Gen = 1000;
No_Population = 300;
Parent_pool = 100;
Elitism_Rate = 0.05;
Crossover_Rate = 0.6;
Permutation_Mutation_Rate = 0.4;
Orientation_Mutation_Rate = 0.4;
ratio = 0.7;
for iteration = 1:2
    if iteration == 1
        Distance_weight = 1;
        Angle_weight = 0;
    else
        Distance_weight = 1;
        Angle_weight = (ratio/(1-ratio))*(solution_distance_cost/solution_angle_cost);
        break % Only for ETSP
    end

%Initialize current generation
current_generation = zeros(No_Population,num_point);
for i=1:No_Population
    current_generation(i,:) = randperm(num_point);
end
CG.Generation = current_generation;
CG.Fitness = zeros(size(current_generation,1),1);
CG.DistanceCost = zeros(size(current_generation,1),1);
CG.AngleCost = zeros(size(current_generation,1),1);

%Find total cost, sort in descending order, For the first generation
for i = 1:No_Population
    distance_cost = 0;
    angle_cost = 0;
    for ii = 1:(num_point-1)
        distance_cost = norm([point(CG.Generation(i,ii+1),1) point(CG.Generation(i,ii+1),2) point(CG.Generation(i,ii+1),3)]-[point(CG.Generation(i,ii),1) point(CG.Generation(i,ii),2) point(CG.Generation(i,ii),3)]) + distance_cost;
        if ii ~= 1
            p1 = [point(CG.Generation(i,ii-1),1) point(CG.Generation(i,ii-1),2)];
            p2 = [point(CG.Generation(i,ii),1) point(CG.Generation(i,ii),2)];
            p3 = [point(CG.Generation(i,ii+1),1) point(CG.Generation(i,ii+1),2)];
            angle_cost = acos(dot(((p2-p1)/norm(p2-p1)), ((p3-p2)/norm(p3-p2)))) + angle_cost;
        end
    end
    distance_cost = norm([point(CG.Generation(i,num_point),1) point(CG.Generation(i,num_point),2) point(CG.Generation(i,num_point),3)]-[point(CG.Generation(i,1),1) point(CG.Generation(i,1),2) point(CG.Generation(i,1),3)]) + distance_cost;
    p1 = [point(CG.Generation(i,num_point-1),1) point(CG.Generation(i,num_point-1),2)];
    p2 = [point(CG.Generation(i,num_point),1) point(CG.Generation(i,num_point),2)];
    p3 = [point(CG.Generation(i,1),1) point(CG.Generation(i,1),2)];
    angle_cost = acos(dot(((p2-p1)/norm(p2-p1)), ((p3-p2)/norm(p3-p2)))) + angle_cost;
    p1 = [point(CG.Generation(i,num_point),1) point(CG.Generation(i,num_point),2)];
    p2 = [point(CG.Generation(i,1),1) point(CG.Generation(i,1),2)];
    p3 = [point(CG.Generation(i,2),1) point(CG.Generation(i,2),2)];
    angle_cost = acos(dot(((p2-p1)/norm(p2-p1)), ((p3-p2)/norm(p3-p2)))) + angle_cost;
    CG.Fitness(i) = 1/(Distance_weight*distance_cost + Angle_weight*angle_cost);
    CG.DistanceCost(i) = distance_cost;
    CG.AngleCost(i) = angle_cost;
end
[~, descend_id] = sort(CG.Fitness,'descend');
CG.Generation = CG.Generation(descend_id,:);
CG.Fitness = CG.Fitness(descend_id,:);

for generation_index = 2:No_Gen
NG.Generation = zeros(No_Population,num_point);
NG.Fitness = zeros(No_Population,1);
NG.DistanceCost = zeros(No_Population,1);
NG.AngleCost = zeros(No_Population,1);
NG.Generation(1:round(No_Population*Elitism_Rate),:) = CG.Generation(1:round(No_Population*Elitism_Rate),:);
NG.Fitness(1:round(No_Population*Elitism_Rate),:) = CG.Fitness(1:round(No_Population*Elitism_Rate),:);
NG.DistanceCost(1:round(No_Population*Elitism_Rate),:) = CG.DistanceCost(1:round(No_Population*Elitism_Rate),:);
NG.AngleCost(1:round(No_Population*Elitism_Rate),:) = CG.AngleCost(1:round(No_Population*Elitism_Rate),:);
for i = round(No_Population*Elitism_Rate)+1 : No_Population
    index = randi([1 No_Population],1,Parent_pool);
    parent_candidate = CG.Fitness(index,:);
    [~, candidate_list] = sort(parent_candidate,'descend');
    index = index(candidate_list);
    parent1 = CG.Generation(index(1),:);
    loop_id = 1;
    while true
        if index(1) ~= index(loop_id+1)
            parent2 = CG.Generation(index(loop_id+1),:);
            break
        end
        loop_id = loop_id + 1;
    end
    if rand <= Crossover_Rate
        child = Crossover(parent1, parent2,num_point,point);
    elseif CG.Fitness(index(1)) > CG.Fitness(index(loop_id+1))
        child = parent1;
    else 
        child = parent2;
    end
    if rand <= Permutation_Mutation_Rate
        child = Reversion_Mutation(child);
    end
    if rand <= Orientation_Mutation_Rate
        child = Reciprocal_Exchange(child);
    end
    NG.Generation(i,:) = child;

    distance_cost = 0;
    angle_cost = 0;
    for ii = 1:(num_point-1)
        distance_cost = norm([point(NG.Generation(i,ii+1),1) point(NG.Generation(i,ii+1),2) point(NG.Generation(i,ii+1),3)]-[point(NG.Generation(i,ii),1) point(NG.Generation(i,ii),2) point(NG.Generation(i,ii),3)]) + distance_cost;
        if ii ~= 1
            p1 = [point(NG.Generation(i,ii-1),1) point(NG.Generation(i,ii-1),2)];
            p2 = [point(NG.Generation(i,ii),1) point(NG.Generation(i,ii),2)];
            p3 = [point(NG.Generation(i,ii+1),1) point(NG.Generation(i,ii+1),2)];
            angle_cost = acos(dot(((p2-p1)/norm(p2-p1)), ((p3-p2)/norm(p3-p2)))) + angle_cost;
        end
    end
    distance_cost = norm([point(NG.Generation(i,num_point),1) point(NG.Generation(i,num_point),2) point(NG.Generation(i,num_point),3)]-[point(NG.Generation(i,1),1) point(NG.Generation(i,1),2) point(NG.Generation(i,1),3)]) + distance_cost;
    p1 = [point(NG.Generation(i,num_point-1),1) point(NG.Generation(i,num_point-1),2)];
    p2 = [point(NG.Generation(i,num_point),1) point(NG.Generation(i,num_point),2)];
    p3 = [point(NG.Generation(i,1),1) point(NG.Generation(i,1),2)];
    angle_cost = acos(dot(((p2-p1)/norm(p2-p1)), ((p3-p2)/norm(p3-p2)))) + angle_cost;
    p1 = [point(NG.Generation(i,num_point),1) point(NG.Generation(i,num_point),2)];
    p2 = [point(NG.Generation(i,1),1) point(NG.Generation(i,1),2)];
    p3 = [point(NG.Generation(i,2),1) point(NG.Generation(i,2),2)];
    angle_cost = acos(dot(((p2-p1)/norm(p2-p1)), ((p3-p2)/norm(p3-p2)))) + angle_cost;
    NG.Fitness(i) = 1/(Distance_weight*distance_cost + Angle_weight*angle_cost);
    NG.DistanceCost(i) = distance_cost;
    NG.AngleCost(i) = angle_cost;
end
[~, descend_id] = sort(NG.Fitness,'descend');
CG.Generation = NG.Generation(descend_id,:);
CG.Fitness = NG.Fitness(descend_id,:);
CG.DistanceCost = NG.DistanceCost(descend_id,:);
CG.AngleCost = NG.AngleCost(descend_id,:);
end
Final_dist_cost = CG.DistanceCost(1);
Final_angle_cost = CG.AngleCost(1);
close_solution = CG.Generation(1,:);
for i = 1:size(close_solution,2)
    if close_solution(i) == num_point
        opening = i;
    end
end
open_solution = zeros(1,num_point-1);
if opening ~= num_point && opening ~= 1
    part1 = close_solution(opening+1:end);
    part2 = close_solution(1:opening-1);
    open_solution = [part1, part2];
elseif opening == 1
    part1 = close_solution(opening+1:end);
    open_solution = part1;
elseif opening == num_point
    part1 = close_solution(1:opening-1);
    open_solution = part1;
end

solution_distance_cost = 0;
solution_angle_cost = 0;
for ii = 1:(num_point-2)
        solution_distance_cost = norm([point(open_solution(ii+1),1) point(open_solution(ii+1),2) point(open_solution(ii+1),3)]-[point(open_solution(ii),1) point(open_solution(ii),2) point(open_solution(ii),3)]) + solution_distance_cost;
        if ii ~= 1
            p1 = [point(open_solution(ii-1),1) point(open_solution(ii-1),2)];
            p2 = [point(open_solution(ii),1) point(open_solution(ii),2)];
            p3 = [point(open_solution(ii+1),1) point(open_solution(ii+1),2)];
            solution_angle_cost = acos(dot(((p2-p1)/norm(p2-p1)), ((p3-p2)/norm(p3-p2)))) + solution_angle_cost;
        end
end
solution_point = [point(open_solution,1) point(open_solution,2) point(open_solution,3)];
end
end