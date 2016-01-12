% Author: Charles ZHU
% --
% Statistics, w/ variable number of DS
% Batch script (mid version)

init_p;

% Initialize environment
clc;
%rand('state', 0); %#ok<RAND>
%randn('state', 0); %#ok<RAND>
rng('default');
rng(0);

% Constants for DS
R_0 = 1500;
S_M = 5000;
S_RANGE = 3000;
DD_M = 80;
D_OFFSET = 120;
DD_RANGE = 120;
LENGTH = 21600;

% Constants for OP
N_OP = 20;
DX_M = 1200;
ER_MU = 500;
ER_SIGMA = 250;
ER_MIN = 25;

% Constants
N_LOOP = 20;

% Random seeds for loops
rng_seeds = randi(2 ^ 32 - 1, N_LOOP, 2);

number_of_ds = (10:10:200)';
dxs_mu = LENGTH./ number_of_ds;
dxs_sigma = dxs_mu./ 3;
nm_ds = size(number_of_ds, 1);
loop_n = N_LOOP * nm_ds;
reward_total = zeros(nm_ds, 3);
time_running = zeros(nm_ds, 3);
rate_total = zeros(nm_ds, 9);
rate_all_total = zeros(nm_ds, 3);
length_task = zeros(nm_ds, 3);

tic
for j = 1:nm_ds
    reward_acc = zeros(1, 3);
    time_acc = zeros(1, 3);
    rate_acc = zeros(3, 6);
    length_acc = zeros(1, 3);
    
    for k = 1:N_LOOP
        % Generate demo instances
        rng(rng_seeds(k, 1));
        v_ds = mk_vec_ds_new(number_of_ds(j), dxs_mu(j), dxs_sigma(j), ...
            R_0, ...
            S_M, S_RANGE, ...
            DD_M, D_OFFSET, DD_RANGE);
        rng(rng_seeds(k, 2));
        v_op = mk_vec_op(N_OP, DX_M, ER_MU, ER_SIGMA, ER_MIN);
        
        loop_j = k + (j - 1)* N_LOOP;
        fprintf(sprintf('Running loop %d of %d...\n', loop_j, loop_n));
        
        % ASAP planning
        et = cputime;
        [mat_m, ~] = plan_asap(v_ds, v_op);
        time_acc(1) = time_acc(1) + (cputime - et);

        % Calculate actual upload time
        t_up = vec_t_up(v_ds, v_op, mat_m, T_WAIT);
        v_f = vec_f(v_ds, t_up);
        t_comp = vec_t_comp(v_ds, v_op, mat_m, T_WAIT); %#ok<*NASGU>
        
        reward_acc(1) = reward_acc(1) + reward(v_ds, v_f);
        rate_acc(1, :) = rate_acc(1, :) + rate_new_row(v_ds, t_up);
        length_acc(1) = length_acc(1) + t_comp(size(t_comp, 1) - 1);
        
        % Algorithm 4 planning
        et = cputime;
        [mat_m, ~] = plan_alg4x(v_ds, v_op, T_WAIT);
        time_acc(2) = time_acc(2) + (cputime - et);
        
        % Calculate actual upload time
        t_up = vec_t_up(v_ds, v_op, mat_m, T_WAIT);
        v_f = vec_f(v_ds, t_up);
        t_comp = vec_t_comp(v_ds, v_op, mat_m, T_WAIT);
        
        reward_acc(2) = reward_acc(2) + reward(v_ds, v_f);
        rate_acc(2, :) = rate_acc(2, :) + rate_new_row(v_ds, t_up);
        length_acc(2) = length_acc(2) + t_comp(size(t_comp, 1) - 1);
        
        % ASAP planning
        [cst_m, cst_ls] = plan_asap(v_ds, v_op);

        % GA planning
        et = cputime;
        [mat_m, ls] = plan_ga(v_ds, v_op, cst_ls, T_WAIT);
        time_acc(3) = time_acc(3) + (cputime - et);
        fprintf('\n');

        % Calculate actual upload time
        t_up = vec_t_up(v_ds, v_op, mat_m, T_WAIT);
        v_f = vec_f(v_ds, t_up);
        t_comp = vec_t_comp(v_ds, v_op, mat_m, T_WAIT);
        
        reward_acc(3) = reward_acc(3) + reward(v_ds, v_f);
        rate_acc(3, :) = rate_acc(3, :) + rate_new_row(v_ds, t_up);
        length_acc(3) = length_acc(3) + t_comp(size(t_comp, 1) - 1);
    end
    reward_total(j, :) = reward_acc / N_LOOP;
    time_running(j, :) = time_acc / N_LOOP;
    rate_total(j, 1:3) = rate_acc(1, 4:6)./ rate_acc(1, 1:3);
    rate_total(j, 4:6) = rate_acc(2, 4:6)./ rate_acc(2, 1:3);
    rate_total(j, 7:9) = rate_acc(3, 4:6)./ rate_acc(3, 1:3);
    rate_all_total(j, :) = [ ...
        sum(rate_acc(1, 4:6)) / sum(rate_acc(1, 1:3)), ...
        sum(rate_acc(2, 4:6)) / sum(rate_acc(2, 1:3)), ...
        sum(rate_acc(3, 4:6)) / sum(rate_acc(3, 1:3))];
    length_task(j, :) = length_acc / N_LOOP;
end
toc
plot(number_of_ds, reward_total(:, 1), ...
    number_of_ds, reward_total(:, 2), '-*', ...
    number_of_ds, reward_total(:, 3), '-o');
xlabel('Number of data sites');
ylabel('Weighted overall utility');
legend('First opportunity', 'Balanced DOP', 'Genetic algorithm');
saveas(gcf, 'fig/mid_ds_number_2_reward.fig');

figure;
plot(number_of_ds, time_running(:, 1), ...
    number_of_ds, time_running(:, 2), '-*', ...
    number_of_ds, time_running(:, 3), '-o');
xlabel('Number of data sites');
ylabel('Running time (sec)');
legend('First opportunity', 'Balanced DOP', 'Genetic algorithm');
saveas(gcf, 'fig/mid_ds_number_2_time.fig');

figure;
plot(number_of_ds, rate_total(:, 1), ...
    number_of_ds, rate_total(:, 4), '-*', ...
    number_of_ds, rate_total(:, 7), '-o');
xlabel('Number of data sites');
ylabel('Portion of important data chunks uploaded');
legend('First opportunity', 'Balanced DOP', 'Genetic algorithm');
saveas(gcf, 'fig_2/mid_ds_number_2_high.fig');

figure;
plot(number_of_ds, rate_total(:, 2), ...
    number_of_ds, rate_total(:, 5), '-*', ...
    number_of_ds, rate_total(:, 8), '-o');
xlabel('Number of data sites');
ylabel('Portion of medium data chunks uploaded');
legend('First opportunity', 'Balanced DOP', 'Genetic algorithm');
saveas(gcf, 'fig_2/mid_ds_number_2_medium.fig');

figure;
plot(number_of_ds, rate_total(:, 3), ...
    number_of_ds, rate_total(:, 6), '-*', ...
    number_of_ds, rate_total(:, 9), '-o');
xlabel('Number of data sites');
ylabel('Portion of unimp. data chunks uploaded');
legend('First opportunity', 'Balanced DOP', 'Genetic algorithm');
saveas(gcf, 'fig_2/mid_ds_number_2_low.fig');

figure;
plot(number_of_ds, rate_all_total(:, 1), ...
    number_of_ds, rate_all_total(:, 2), '-*', ...
    number_of_ds, rate_all_total(:, 3), '-o');
xlabel('Number of data sites');
ylabel('Portion of data chunks uploaded');
legend('First opportunity', 'Balanced DOP', 'Genetic algorithm');
saveas(gcf, 'fig_2/mid_ds_number_2_all.fig');

figure;
plot(number_of_ds, length_task(:, 1), ...
    number_of_ds, length_task(:, 2), '-*', ...
    number_of_ds, length_task(:, 3), '-o');
xlabel('Number of data sites');
ylabel('Time to complete all data collection (sec)');
legend('First opportunity', 'Balanced DOP', 'Genetic algorithm');
saveas(gcf, 'fig_2/mid_ds_number_2_length.fig');

save('mat/mid_ds_number_2.mat')
