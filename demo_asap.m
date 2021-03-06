% Author: Charles ZHU
% --
% Demo for ASAP planning (ASAP upload) and objective function calculation
% Compatible with simple (old fashion) and rich (new fashion) DS configs

% Initialize environment
clc;
rand('state', 3); %#ok<RAND>
randn('state', 3); %#ok<RAND>

% Generate demo instances
if exist('S_0', 'var') % Old fashion
    v_ds = mk_vec_ds(N_DS, DX_MU, DX_SIGMA, R_0, S_0, DD_M);
else % New fashion
    if exist('D_OFFSET', 'var')
        if exist('DD_RANGE', 'var')
            v_ds = mk_vec_ds_new( ...
                N_DS, DX_MU, DX_SIGMA, R_0, ...
                S_M, S_RANGE, ...
                DD_M, D_OFFSET, DD_RANGE);
        else % Using default DD_RANGE, which is DD_M
            v_ds = mk_vec_ds_new( ...
                N_DS, DX_MU, DX_SIGMA, R_0, ...
                S_M, S_RANGE, ...
                DD_M, D_OFFSET);
        end
    else % Using default D_OFFSET, which is 0
        v_ds = mk_vec_ds_new( ...
            N_DS, DX_MU, DX_SIGMA, R_0, ...
            S_M, S_RANGE, ...
            DD_M);
    end
end
v_op = mk_vec_op(N_OP, DX_M, ER_MU, ER_SIGMA, ER_MIN);

% ASAP planning
fprintf('ASAP planning...\n\n');
tic
[mat_m, ls] = plan_asap(v_ds, v_op);
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
