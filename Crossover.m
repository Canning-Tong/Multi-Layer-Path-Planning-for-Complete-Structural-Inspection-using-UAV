%Crossover Function 2D
function child = Crossover(parent1, parent2, num_point, point)
    child = zeros(1,num_point);
    current = randi([1 num_point],1);
    child(1) = current;
    for i = 2:num_point
            repeat_flag1 = 0;
            repeat_flag2 = 0;
            for ii = 1:num_point
                if parent1(ii) == current
                    parent1_id = parent1(ii);
                    if ii == num_point
                        parent1_id2 = parent1(1);
                    else
                        parent1_id2 = parent1(ii+1);
                    end
                end
                if parent2(ii) == current
                    parent2_id = parent2(ii);
                    if ii == num_point
                        parent2_id2 = parent2(1);
                    else
                        parent2_id2 = parent2(ii+1);
                    end
                end
            end
            parent1_dist = norm([point(parent1_id2,1) point(parent1_id2,2)] - [point(parent1_id,1) point(parent1_id,2)]);
            parent2_dist = norm([point(parent2_id2,1) point(parent2_id2,2)] - [point(parent2_id,1) point(parent2_id,2)]);
            for ii = 1:size(child,2)
                if child(ii) == parent1_id2
                    repeat_flag1 = 1;
                end
                if child(ii) == parent2_id2
                    repeat_flag2 = 1;
                end
            end
            if (parent1_dist < parent2_dist && repeat_flag1 == 0) || (repeat_flag1 == 0 && repeat_flag2 == 1)
                    child(i) = parent1_id2;
                    current = parent1_id2;
                    continue
            end
            if (parent2_dist < parent1_dist && repeat_flag2 == 0) || (repeat_flag1 == 1 && repeat_flag2 == 0)
                    child(i) = parent2_id2;
                    current = parent2_id2;
                    continue
            end
            if parent1_dist == parent2_dist || (repeat_flag1 == 1 && repeat_flag2 == 1)
                if repeat_flag1 == 0 && repeat_flag2 == 1
                    child(i) = parent1_id2;
                    current = parent1_id2;
                    continue
                elseif repeat_flag2 == 0 && repeat_flag1 == 1
                    child(i) = parent2_id2;
                    current = parent2_id2;
                    continue
                elseif repeat_flag1 == 0 && repeat_flag2 == 0
                    child(i) = parent1_id2;
                    current = parent1_id2;
                    continue
                elseif repeat_flag1 == 1 && repeat_flag2 == 1
                    while true
                        repeat_flag3 = 0;
                        current = randi([1 num_point],1);
                        for ii = 1:size(child,2)
                            if child(ii) == current
                                repeat_flag3 = 1;
                            end
                        end
                        if repeat_flag3 == 0
                            child(i) = current;
                            break
                        end
                    end
                end
            end
    end
end