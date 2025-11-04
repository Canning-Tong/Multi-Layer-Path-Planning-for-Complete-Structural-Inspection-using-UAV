%Reversion Mutation Function
function child = Reversion_Mutation(old_child)
    index = [randi([2 floor((size(old_child,2)/2))],1) randi([floor((size(old_child,2)/2))+1 (size(old_child,2)-1)],1)];
    content = flip(old_child(index(1):index(2)));
    child = old_child;
    child(index(1):index(2)) = content;
end