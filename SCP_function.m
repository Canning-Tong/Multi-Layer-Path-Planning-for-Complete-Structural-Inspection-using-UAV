%SCP function%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [SCP_solution] = SCP_function(Overall_visibility, Viewpoint, LowG_flag, mesh_overlap)
constraints = zeros(size(Overall_visibility,1), size(Viewpoint.v,2));
overall_vis_size = size(Overall_visibility,1);
vp_size = size(Viewpoint.v,2);

%%%%% Original %%%%%
% parfor i = 1:overall_vis_size %Loop for all viewpoint
%%%%% Update %%%%%
for i = 1:overall_vis_size %Loop for all viewpoint
    for ii = 1:vp_size %Loop for visibility of viewpoint
        if Viewpoint.v(ii).visibility(i) > 0
            constraints(i,ii) = 1;
        end
    end
end
ctsr_id = zeros(overall_vis_size,1);
for i = 1:overall_vis_size
    if Overall_visibility(i) == LowG_flag
        ctsr_id(i) = 1;
    end
end
ctsr_id = logical(ctsr_id);
constraints(ctsr_id,:) = [];
set_cover = optimproblem;
vp = optimvar('vp', size(Viewpoint.v,2), 'Type','integer', 'LowerBound',0, 'UpperBound',1);
set_cover.Objective = sum(vp);
meshconstr = optimconstr(size(constraints,1),1);
constrsize = size(meshconstr,1);
for i = 1:constrsize
    meshconstr(i) = constraints(i,:)*(vp) >= mesh_overlap;
end
set_cover.Constraints.mesh = meshconstr;
problem = prob2struct(set_cover);
problem.options.MaxTime = 600;
[SCP_solution,fval,exitflag,output] = intlinprog(problem);
SCP_solution = logical(round(SCP_solution));