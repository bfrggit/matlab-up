% Author: Charles ZHU
% --
% Demo for brute force search compared to ASAP planning

% Initialize environment
clc;
rand('state', 3); %#ok<RAND>
randn('state', 3); %#ok<RAND>

% Generate demo instances
v_ds = mk_vec_ds_new(N_DS, DX_MU, DX_SIGMA, R_0, S_M, S_RANGE, DD_M, D_OFFSET);
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
rw = reward(v_ds, v_f) %#ok<NOPTS,NASGU>
fprintf('\nCount for each priority group');
ss = rate(v_ds, t_up) %#ok<NOPTS>
fprintf('\nSuccess rate for each priority group');
sr = ss(:, 2)./ ss(:, 1) %#ok<NOPTS,NASGU>

% Brute force search planning
fprintf('Brute force search planning...\n\n');
tic
[mat_m, ls] = plan_best(v_ds, v_op, T_WAIT);
fprintf('\n');
toc

% Calculate actual upload time
t_up = vec_t_up(v_ds, v_op, mat_m, T_WAIT);
v_f = vec_f(v_ds, t_up);

fprintf('\nReward of plan');
rw = reward(v_ds, v_f) %#ok<NOPTS>
fprintf('\nCount for each priority group');
ss = rate(v_ds, t_up) %#ok<NOPTS>
fprintf('\nSuccess rate for each priority group');
sr = ss(:, 2)./ ss(:, 1) %#ok<NOPTS>
