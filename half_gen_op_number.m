% Author: Charles ZHU
% --
% Statistics, w/ fixed path length, variable number of OP
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
S_M = 5000;
S_RANGE = 3000;
DD_M = 80;
D_OFFSET = 120;
DD_RANGE = 120;

% Constants for OP
LENGTH = 24000;
ER_MU = 500;
ER_SIGMA = 250;
ER_MIN = 25;

% Constants
N_LOOP = 10;

number_of_op = (4:4:40)';
dxs_m = LENGTH./ number_of_op;
nm_op = size(number_of_op, 1);
loop_n = N_LOOP * nm_op;
reward_total = zeros(nm_op, 3);
time_running = zeros(nm_op, 3);
rate_total = zeros(nm_op, 9);
rate_all_total = zeros(nm_op, 3);

mkdir('half');
mkdir('half', 'change_op_number');

tic
for j = 1:nm_op
    rw1_total = 0.0; rw2_total = 0.0; rw3_total = 0.0;
    et_plan1 = 0.0; et_plan2 = 0.0; et_plan3 = 0.0;
    rt1_total = zeros(1, 6); rt2_total = rt1_total; rt3_total = rt1_total;
    
    mkdir('half/change_op_number', sprintf('%d', number_of_op(j)));
    
    for k = 1:N_LOOP
        mkdir(sprintf('half/change_op_number/%d', number_of_op(j)), ...
            sprintf('case_%d', k));
        
        % Generate demo instances
        v_ds = mk_vec_ds_new(N_DS, DX_MU, DX_SIGMA, R_0, S_M, S_RANGE, DD_M, D_OFFSET, DD_RANGE);
        v_op = mk_vec_op(number_of_op(j), dxs_m(j), ER_MU, ER_SIGMA, ER_MIN);
        
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
        rt1 = rate_new_row(v_ds, t_up);
        rt1_total = rt1_total + rt1;
        
        save(sprintf('half/change_op_number/%d/case_%d/asap.mat', ...
                number_of_op(j), k));
        
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
        rt2 = rate_new_row(v_ds, t_up);
        rt2_total = rt2_total + rt2;
        
        save(sprintf('half/change_op_number/%d/case_%d/alg4.mat', ...
                number_of_op(j), k));
        
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
        rt3 = rate_new_row(v_ds, t_up);
        rt3_total = rt3_total + rt3;
        
        save(sprintf('half/change_op_number/%d/case_%d/ga.mat', ...
                number_of_op(j), k));
    end
    reward_total(j, :) = [rw1_total, rw2_total, rw3_total] / N_LOOP;
    time_running(j, :) = [et_plan1, et_plan2, et_plan3] / N_LOOP;
    rate_total(j, 1:3) = rt1_total(4:6)./ rt1_total(1:3);
    rate_total(j, 4:6) = rt2_total(4:6)./ rt2_total(1:3);
    rate_total(j, 7:9) = rt3_total(4:6)./ rt3_total(1:3);
    rate_all_total(j, :) = [ ...
        sum(rt1_total(4:6)) / sum(rt1_total(1:3)), ...
        sum(rt2_total(4:6)) / sum(rt2_total(1:3)), ...
        sum(rt3_total(4:6)) / sum(rt3_total(1:3))];
end
toc
plot(number_of_op, reward_total(:, 1), ...
    number_of_op, reward_total(:, 2), '-*', ...
    number_of_op, reward_total(:, 3), '-o');
xlabel('Number of upload opportunities');
ylabel('Weighted overall utility');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig/half_op_number_reward.fig');

figure;
plot(number_of_op, time_running(:, 1), ...
    number_of_op, time_running(:, 2), '-*', ...
    number_of_op, time_running(:, 3), '-o');
xlabel('Number of upload opportunities');
ylabel('Running time (sec)');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig/half_op_number_time.fig');

figure;
plot(number_of_op, rate_total(:, 1), ...
    number_of_op, rate_total(:, 4), '-*', ...
    number_of_op, rate_total(:, 7), '-o');
xlabel('Number of upload opportunities');
ylabel('Portion of high priority data chunks uploaded');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig_2/half_op_number_high.fig');

figure;
plot(number_of_op, rate_total(:, 2), ...
    number_of_op, rate_total(:, 5), '-*', ...
    number_of_op, rate_total(:, 8), '-o');
xlabel('Number of upload opportunities');
ylabel('Portion of medium priority data chunks uploaded');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig_2/half_op_number_medium.fig');

figure;
plot(number_of_op, rate_total(:, 3), ...
    number_of_op, rate_total(:, 6), '-*', ...
    number_of_op, rate_total(:, 9), '-o');
xlabel('Number of upload opportunities');
ylabel('Portion of low priority data chunks uploaded');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig_2/half_op_number_low.fig');

figure;
plot(number_of_op, rate_all_total(:, 1), ...
    number_of_op, rate_all_total(:, 2), '-*', ...
    number_of_op, rate_all_total(:, 3), '-o');
xlabel('Number of upload opportunities');
ylabel('Portion of data chunks uploaded');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig_2/half_op_number_all.fig');

save('mat/half_op_number.mat')
