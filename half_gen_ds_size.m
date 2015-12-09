% Author: Charles ZHU
% --
% Statistics, w/ fixed number of DS, variable size of DS
% Half workspace generation script for dynamic simulations

init_p;

% Initialize environment
clc;
rand('state', 0); %#ok<RAND>
randn('state', 0); %#ok<RAND>

% Constants for DS
N_DS = 120;
DX_MU = 180;
DX_SIGMA = 60;
R_0 = 1500;
%S_M = 5000;
%S_RANGE = 3000;
DD_M = 80;
D_OFFSET = 120;
DD_RANGE = 120;

% Constants for OP
N_OP = 20;
DX_M = 1200;
ER_MU = 500;
ER_SIGMA = 250;
ER_MIN = 25;

% Constants
N_LOOP = 10;

size_of_ds = (1000:1000:10000)';
ss_range = size_of_ds * 0.6;
nm_ds = size(size_of_ds, 1);
loop_n = N_LOOP * nm_ds;
reward_total = zeros(nm_ds, 3);
time_running = zeros(nm_ds, 3);

mkdir('half');
mkdir('half', 'change_ds_size');

tic
for j = 1:nm_ds
    rw1_total = 0.0; rw2_total = 0.0; rw3_total = 0.0;
    et_plan1 = 0.0; et_plan2 = 0.0; et_plan3 = 0.0;
    
    mkdir('half/change_ds_size', sprintf('%d', size_of_ds(j)));
    
    for k = 1:N_LOOP
        mkdir(sprintf('half/change_ds_size/%d', size_of_ds(j)), ...
            sprintf('case_%d', k));
        
        % Generate demo instances
        v_ds = mk_vec_ds_new(N_DS, DX_MU, DX_SIGMA, R_0, ...
            size_of_ds(j), ss_range(j), DD_M, ...
            D_OFFSET, DD_RANGE);
        v_op = mk_vec_op(N_OP, DX_M, ER_MU, ER_SIGMA, ER_MIN);
        
        loop_j = k + (j - 1)* N_LOOP;
        fprintf(sprintf('Running loop %d of %d...\n', loop_j, loop_n));
        
        % ASAP planning
        et = cputime;
        [mat_m, ~] = plan_asap(v_ds, v_op);
        et_plan1 = et_plan1 + (cputime - et);

        % Calculate actual upload time
        t_up = vec_t_up(v_ds, v_op, mat_m, T_WAIT);
        v_f = vec_f(v_ds, t_up);
        t_comp = vec_t_comp(v_ds, v_op, mat_m, T_WAIT); %#ok<*NASGU>
        
        rw1 = reward(v_ds, v_f);
        rw1_total = rw1_total + rw1;
        
        save(sprintf('half/change_ds_size/%d/case_%d/asap.mat', ...
                size_of_ds(j), k));
        
        % Algorithm 4 planning
        et = cputime;
        [mat_m, ~] = plan_alg4x(v_ds, v_op, T_WAIT);
        et_plan2 = et_plan2 + (cputime - et);
        
        % Calculate actual upload time
        t_up = vec_t_up(v_ds, v_op, mat_m, T_WAIT);
        v_f = vec_f(v_ds, t_up);
        t_comp = vec_t_comp(v_ds, v_op, mat_m, T_WAIT);
        
        rw2 = reward(v_ds, v_f);
        rw2_total = rw2_total + rw2;
        
        save(sprintf('half/change_ds_size/%d/case_%d/alg4.mat', ...
                size_of_ds(j), k));
        
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
        t_comp = vec_t_comp(v_ds, v_op, mat_m, T_WAIT);
        
        rw3 = reward(v_ds, v_f);
        rw3_total = rw3_total + rw3;
        
        save(sprintf('half/change_ds_size/%d/case_%d/ga.mat', ...
                size_of_ds(j), k));
    end
    reward_total(j, 1) = rw1_total / N_LOOP;
    reward_total(j, 2) = rw2_total / N_LOOP;
    reward_total(j, 3) = rw3_total / N_LOOP;
    time_running(j, 1) = et_plan1 / N_LOOP;
    time_running(j, 2) = et_plan2 / N_LOOP;
    time_running(j, 3) = et_plan3 / N_LOOP;
end
toc
plot(size_of_ds, reward_total(:, 1), ...
    size_of_ds, reward_total(:, 2), '-*', ...
    size_of_ds, reward_total(:, 3), '-o');
xlabel('Size of one single data chunk (kB)');
ylabel('Weighted overall utility');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig/half_ds_size_reward.fig');

figure;
plot(size_of_ds, time_running(:, 1), ...
    size_of_ds, time_running(:, 2), '-*', ...
    size_of_ds, time_running(:, 3), '-o');
xlabel('Size of one single data chunk (kB)');
ylabel('Running time (sec)');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig/half_ds_size_time.fig');

save('mat/half_ds_size.mat')
