% Author: Charles ZHU
% --
% Demo for ASAP planning (ASAP upload) and objective function calculation

% Initialize environment
init_p;
rand('state', 0); %#ok<RAND>
randn('state', 0); %#ok<RAND>

% Constants
N_DS = 20; DX_MU = 500; DX_SIGMA = 100; R_0 = 1000; S_0 = 10000; DD_M = 250;
N_OP = 50; DX_M = 200; ER_MU = 500; ER_SIGMA = 200;

% Generate demo instances
v_ds = mk_vec_ds(N_DS, DX_MU, DX_SIGMA, R_0, S_0, DD_M);
v_op = mk_vec_op(N_OP, DX_M, ER_MU, ER_SIGMA);

% ASAP planning
tic
mat_m = plan_asap(v_ds, v_op);
toc

% Calculate actual upload time
t_up = vec_t_up(v_ds, v_op, mat_m);
v_f = vec_f(v_ds, t_up);

fprintf('\nReward of plan');
rw = reward(v_ds, v_f)
fprintf('\nCount for each priority group');
ss = rate(v_ds, t_up)
fprintf('\nSuccess rate for each priority group');
sr = ss(:, 2)./ ss(:, 1)
