function [m, ls] = plan_ga(v_ds, v_op, cst_ls)
%PLAN_GA        Generate a plan using GA planning (genetic algorithm)
%PLAN_GA(v_ds, v_op, cst_ls)
%   v_ds        DS vector
%   v_op        OP vector
%   cst_ls      Constraint vector (ASAP planning)

% GA functions
n_ds = size(v_ds, 1);
n_op = size(v_op, 1);
fitness = @(ls) 0 - reward(v_ds, vec_f(v_ds, vec_t_up(v_ds, v_op, ls_to_m(ls', n_op))));
lb = cst_ls';
ub = repmat(n_op, 1, n_ds);
opts = gaoptimset('TolFun', 1e-6, ...
    'InitialPopulation', cst_ls' ...
    );

[xbest, fbest, exitflag] = ga( ...
    @(ls) fitness(ls), ...
    n_ds, ...
    [], [], [], [], ...
    lb, ub, ...
    [], ...
    1:n_ds, ...
    opts); %#ok<NASGU,ASGLU>
ls = xbest';
m = ls_to_m(ls, n_op);

end

