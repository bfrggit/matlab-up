% Author: Charles ZHU
% --
% Statistics, w/ fixed path length, variable number of OP
% Batch script

init_p;

% Initialize environment
clc;
rand('state', 0); %#ok<RAND>
randn('state', 0); %#ok<RAND>

% Constants for DS
N_DS = 30;
DX_MU = 180;
DX_SIGMA = 60;
R_0 = 1000;
S_0 = 5000;
DD_M = 60;

% Constants for OP
LENGTH = 6000;
ER_MU = 500;
ER_SIGMA = 250;
ER_MIN = 25;

% Constants
N_LOOP = 50;

number_of_op = (5:5:100)';
dxs_m = LENGTH./ number_of_op;
nm_op = size(number_of_op, 1);
loop_n = N_LOOP * nm_op;
reward_total = zeros(nm_op, 3);
time_running = zeros(nm_op, 3);

tic
for j = 1:nm_op
    rw1_total = 0.0;
    rw2_total = 0.0;
    rw3_total = 0.0;
    et_plan1 = 0.0;
    et_plan2 = 0.0;
    et_plan3 = 0.0;
    for k = 1:N_LOOP
        % Generate demo instances
        v_ds = mk_vec_ds(N_DS, DX_MU, DX_SIGMA, R_0, S_0, DD_M);
        v_op = mk_vec_op(number_of_op(j), dxs_m(j), ER_MU, ER_SIGMA, ER_MIN);
        
        loop_j = k + (j - 1)* N_LOOP;
        fprintf(sprintf('Running loop %d of %d...\n', loop_j, loop_n));
        
        % ASAP planning
        et = cputime;
        [mat_m, ls] = plan_asap(v_ds, v_op);
        et_plan1 = et_plan1 + (cputime - et);

        % Calculate actual upload time
        t_up = vec_t_up(v_ds, v_op, mat_m, T_WAIT);
        v_f = vec_f(v_ds, t_up);
        
        rw1 = reward(v_ds, v_f);
        rw1_total = rw1_total + rw1;
        
        % Algorithm 4 planning
        et = cputime;
        [mat_m, ls] = plan_alg4(v_ds, v_op, T_WAIT);
        et_plan2 = et_plan2 + (cputime - et);
        
        % Calculate actual upload time
        t_up = vec_t_up(v_ds, v_op, mat_m, T_WAIT);
        v_f = vec_f(v_ds, t_up);
        
        rw2 = reward(v_ds, v_f);
        rw2_total = rw2_total + rw2;
        
        % ASAP planning
        [cst_m, cst_ls] = plan_asap(v_ds, v_op);

        % GA planning
        et = cputime;
        [mat_m, ls] = plan_ga(v_ds, v_op, cst_ls, T_WAIT);
        et_plan3 = et_plan3 + (cputime - et);
        fprintf('\n');

        % Calculate actual upload time
        t_up = vec_t_up(v_ds, v_op, mat_m, T_WAIT);
        v_f = vec_f(v_ds, t_up);
        
        rw3 = reward(v_ds, v_f);
        rw3_total = rw3_total + rw3;
    end
    reward_total(j, 1) = rw1_total / N_LOOP;
    reward_total(j, 2) = rw2_total / N_LOOP;
    reward_total(j, 3) = rw3_total / N_LOOP;
    time_running(j, 1) = et_plan1 / N_LOOP;
    time_running(j, 2) = et_plan2 / N_LOOP;
    time_running(j, 3) = et_plan3 / N_LOOP;
end
toc
plot(number_of_op, reward_total);
xlabel('Number of upload opportunities');
ylabel('Weighted overall utility');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
savefig('fig/change_op_number_reward.fig');

figure;
plot(number_of_op, time_running);
xlabel('Number of upload opportunities');
ylabel('Running time (sec)');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
savefig('fig/change_op_number_time.fig');

save('mat/change_op_number.mat')
