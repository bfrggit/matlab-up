% Author: Charles ZHU
% --
% Demo for algorithm 4 compared to ASAP planning

% Initialize environment
clc;
rand('state', 3); %#ok<RAND>
randn('state', 3); %#ok<RAND>

% Generate demo instances
v_ds = mk_vec_ds(N_DS, DX_MU, DX_SIGMA, R_0, S_0, DD_M);
v_op = mk_vec_op(N_OP, DX_M, ER_MU, ER_SIGMA, ER_MIN);

% ASAP planning
fprintf('ASAP planning...\n\n');
tic
[cst_m, cst_ls] = plan_asap(v_ds, v_op);
toc

% Calculate actual upload time
t_up = vec_t_up(v_ds, v_op, cst_m, T_WAIT);
v_f = vec_f(v_ds, t_up);

fprintf('\nReward of plan');
rw = reward(v_ds, v_f) %#ok<NASGU>
fprintf('\nCount for each priority group');
ss = rate(v_ds, t_up)
fprintf('\nSuccess rate for each priority group');
sr = ss(:, 2)./ ss(:, 1) %#ok<NASGU>

% Algorithm 4 planning
fprintf('Algorithm 4 planning: ');
tic
[mat_m, ls] = plan_alg4(v_ds, v_op, T_WAIT);
fprintf('\n');
toc

% Calculate actual upload time
t_up = vec_t_up(v_ds, v_op, mat_m, T_WAIT);
v_f = vec_f(v_ds, t_up);

fprintf('\nReward of plan');
rw = reward(v_ds, v_f)
fprintf('\nCount for each priority group');
ss = rate(v_ds, t_up)
fprintf('\nSuccess rate for each priority group');
sr = ss(:, 2)./ ss(:, 1)
