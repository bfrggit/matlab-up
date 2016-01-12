% Author: Charles ZHU
% --
% Statistics, w/ fixed number of DS, variable size of DS
% Half workspace generation script for dynamic simulations

init_p;

% Initialize environment
clc;
%rand('state', 0); %#ok<RAND>
%randn('state', 0); %#ok<RAND>
rng('default');
rng(0);

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

% Random seeds for loops
rng_seeds = randi(2 ^ 32 - 1, N_LOOP, 2);

size_of_ds = (1000:1000:10000)';
ss_range = size_of_ds * 0.6;
nm_ds = size(size_of_ds, 1);
loop_n = N_LOOP * nm_ds;
reward_total = zeros(nm_ds, 3);
time_running = zeros(nm_ds, 3);
rate_total = zeros(nm_ds, 9);
rate_all_total = zeros(nm_ds, 3);
length_task = zeros(nm_ds, 3);

mkdir('half');
mkdir('half', 'change_ds_size');

tic
for j = 1:nm_ds
    reward_acc = zeros(1, 3);
    time_acc = zeros(1, 3);
    rate_acc = zeros(3, 6);
    length_acc = zeros(1, 3);
    
    mkdir('half/change_ds_size', sprintf('%d', size_of_ds(j)));
    
    for k = 1:N_LOOP
        mkdir(sprintf('half/change_ds_size/%d', size_of_ds(j)), ...
            sprintf('case_%d', k));
        
        % Generate demo instances
        rng(rng_seeds(k, 1));
        v_ds = mk_vec_ds_new(N_DS, DX_MU, DX_SIGMA, R_0, ...
            size_of_ds(j), ss_range(j), ...
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
        
        save(sprintf('half/change_ds_size/%d/case_%d/asap.mat', ...
                size_of_ds(j), k));
        
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
        
        save(sprintf('half/change_ds_size/%d/case_%d/alg4.mat', ...
                size_of_ds(j), k));
        
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
        
        save(sprintf('half/change_ds_size/%d/case_%d/ga.mat', ...
                size_of_ds(j), k));
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
plot(size_of_ds, reward_total(:, 1), ...
    size_of_ds, reward_total(:, 2), '-*', ...
    size_of_ds, reward_total(:, 3), '-o');
xlabel('Average size of data chunks (KB)');
ylabel('Weighted overall utility');
legend('First opportunity', 'Balanced DOP', 'Genetic algorithm', ...
	'Location', 'southwest');
saveas(gcf, 'fig/half_ds_size_reward.fig');

figure;
plot(size_of_ds, time_running(:, 1), ...
    size_of_ds, time_running(:, 2), '-*', ...
    size_of_ds, time_running(:, 3), '-o');
xlabel('Average size of data chunks (KB)');
ylabel('Running time (sec)');
legend('First opportunity', 'Balanced DOP', 'Genetic algorithm', ...
	'Location', 'northwest');
saveas(gcf, 'fig/half_ds_size_time.fig');

figure;
plot(size_of_ds, rate_total(:, 1), ...
    size_of_ds, rate_total(:, 4), '-*', ...
    size_of_ds, rate_total(:, 7), '-o');
xlabel('Average size of data chunks (KB)');
ylabel('Portion of important data chunks uploaded');
legend('First opportunity', 'Balanced DOP', 'Genetic algorithm', ...
	'Location', 'southwest');
saveas(gcf, 'fig_2/half_ds_size_high.fig');

figure;
plot(size_of_ds, rate_total(:, 2), ...
    size_of_ds, rate_total(:, 5), '-*', ...
    size_of_ds, rate_total(:, 8), '-o');
xlabel('Average size of data chunks (KB)');
ylabel('Portion of medium data chunks uploaded');
legend('First opportunity', 'Balanced DOP', 'Genetic algorithm', ...
	'Location', 'southwest');
saveas(gcf, 'fig_2/half_ds_size_medium.fig');

figure;
plot(size_of_ds, rate_total(:, 3), ...
    size_of_ds, rate_total(:, 6), '-*', ...
    size_of_ds, rate_total(:, 9), '-o');
xlabel('Average size of data chunks (KB)');
ylabel('Portion of unimp. data chunks uploaded');
legend('First opportunity', 'Balanced DOP', 'Genetic algorithm', ...
	'Location', 'southwest');
saveas(gcf, 'fig_2/half_ds_size_low.fig');

figure;
plot(size_of_ds, rate_all_total(:, 1), ...
    size_of_ds, rate_all_total(:, 2), '-*', ...
    size_of_ds, rate_all_total(:, 3), '-o');
xlabel('Average size of data chunks (KB)');
ylabel('Portion of data chunks uploaded');
legend('First opportunity', 'Balanced DOP', 'Genetic algorithm', ...
	'Location', 'southwest');
saveas(gcf, 'fig_2/half_ds_size_all.fig');

figure;
plot(size_of_ds, length_task(:, 1), ...
    size_of_ds, length_task(:, 2), '-*', ...
    size_of_ds, length_task(:, 3), '-o');
xlabel('Average size of data chunks (KB)');
ylabel('Time to complete all data collection (sec)');
legend('First opportunity', 'Balanced DOP', 'Genetic algorithm', ...
	'Location', 'northwest');
saveas(gcf, 'fig_2/half_ds_size_length.fig');

save('mat/half_ds_size.mat')
