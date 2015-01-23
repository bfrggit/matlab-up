% Author: Charles ZHU
% --
% Demo for ASAP planning (ASAP upload) and objective function calculation
% Batch script

% Initialize environment
init_p;
rand('state', 0); %#ok<RAND>
randn('state', 0); %#ok<RAND>

% Constants
N_DS = 20; DX_MU = 500; DX_SIGMA = 100; R_0 = 1000; S_0 = 10000; DD_M = 250;
N_OP = 50; DX_M = 200; ER_MU = 500; ER_SIGMA = 200;
N_LOOP = 1000;

rw = zeros(1, N_LOOP);
ss = zeros(size(P_DIST, 1), 2);
et_plan = 0.0;
tic
for j = 1:N_LOOP
    % Generate demo instances
    v_ds = mk_vec_ds(N_DS, DX_MU, DX_SIGMA, R_0, S_0, DD_M);
    v_op = mk_vec_op(N_OP, DX_M, ER_MU, ER_SIGMA);

    % ASAP planning
    et = cputime;
    mat_m = plan_asap(v_ds, v_op);
    et_plan = et_plan + (cputime - et);

    % Calculate actual upload time
    t_up = vec_t_up(v_ds, v_op, mat_m);
    v_f = vec_f(v_ds, t_up);
    
    rw(j) = reward(v_ds, v_f);
    ss = ss + rate(v_ds, t_up);
end
toc

fprintf('\nTime spent in planning is %f seconds.\n', et_plan);
fprintf('\nAverage reward over %d loop(s)', N_LOOP);
rate = sum(rw') / size(rw, 2)
fprintf('\nCount for each priority group');
ss
fprintf('\nSuccess rate for each priority group');
sr = ss(:, 2)./ ss(:, 1)
