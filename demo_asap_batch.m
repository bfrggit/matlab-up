% Author: Charles ZHU
% --
% Demo for ASAP planning (ASAP upload) and objective function calculation
% Batch script

% Initialize environment
clc;
rand('state', 0); %#ok<RAND>
randn('state', 0); %#ok<RAND>

% Constants
N_LOOP = 200;

rw = zeros(1, N_LOOP);
ss = zeros(size(P_DIST, 1), 2);
et_plan = 0.0;
tic
for j = 1:N_LOOP
    % Generate demo instances
    v_ds = mk_vec_ds(N_DS, DX_MU, DX_SIGMA, R_0, S_0, DD_M);
    v_op = mk_vec_op(N_OP, DX_M, ER_MU, ER_SIGMA, ER_MIN);

    % ASAP planning
    et = cputime;
    [mat_m, ls] = plan_asap(v_ds, v_op);
    et_plan = et_plan + (cputime - et);

    % Calculate actual upload time
    t_up = vec_t_up(v_ds, v_op, mat_m, T_WAIT);
    v_f = vec_f(v_ds, t_up);
    
    rw(j) = reward(v_ds, v_f);
    ss = ss + rate(v_ds, t_up);
end
toc

fprintf('\nTime spent in planning is %f seconds.\n', et_plan);
fprintf('\nAverage reward over %d loop(s)', N_LOOP);
%ra = sum(rw, 2) / size(rw, 2)
ra = mean(rw, 2)
hist(rw, 0:0.1:1);
axis([-0.2 1.2 0 160]);
fprintf('\nStandard deviation of reward');
rd = std(rw)
fprintf('\nCount for each priority group');
ss
fprintf('\nSuccess rate for each priority group');
sr = ss(:, 2)./ ss(:, 1)
