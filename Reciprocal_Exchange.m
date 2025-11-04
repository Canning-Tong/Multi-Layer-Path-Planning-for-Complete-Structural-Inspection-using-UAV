%Reciprocal Exchange Function
function child = Reciprocal_Exchange(old_child)
    while true
        index = randi([1 size(old_child,2)],1,2);
        if index(1) ~= index(2)
            break
        end
    end
    content = [old_child(index(1)) old_child(index(2))];
    child = old_child;
    child(index(1)) = content(2);
    child(index(2)) = content(1);
end