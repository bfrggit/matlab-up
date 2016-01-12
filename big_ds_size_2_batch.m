% Author: Charles ZHU
% --
% Statistics, w/ fixed total amount of data, variable size of DS
% Batch script (big version)

init_p;

% Initialize environment
clc;
%rand('state', 0); %#ok<RAND>
%randn('state', 0); %#ok<RAND>
rng('default');
rng(0);

% Constants for DS
R_0 = 1500;
DD_M = 80;
D_OFFSET = 120;
DD_RANGE = 120;
TOTAL_SIZE = 5000000;
LENGTH = 180000;

% Constants for OP
N_OP = 40;
DX_M = 5000;
ER_MU = 500;
ER_SIGMA = 250;
ER_MIN = 25;

% Constants
N_LOOP = 10;

% Random seeds for loops
rng_seeds = randi(2 ^ 32 - 1, N_LOOP, 2);

size_of_ds = (500:500:10000)';
ss_range = size_of_ds * 0.6;
ns_ds = round(TOTAL_SIZE./ size_of_ds);
dxs_mu = LENGTH./ ns_ds;
dxs_sigma = dxs_mu./ 3;
nm_ds = size(size_of_ds, 1);
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
        v_ds = mk_vec_ds_new(ns_ds(j), dxs_mu(j), dxs_sigma(j), R_0, ...
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

        fprintf('\n');
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
    size_of_ds, reward_total(:, 2), '-*'); %, ...
    %size_of_ds, reward_total(:, 3), '-o');
xlabel('Average size of data chunks (KB)');
ylabel('Weighted overall utility');
legend('First opportunity', 'Balanced DOP', ...
    'Location', 'southwest');
saveas(gcf, 'fig/big_ds_size_2_reward.fig');

figure;
plot(size_of_ds, time_running(:, 1), ...
    size_of_ds, time_running(:, 2), '-*'); %, ...
    %size_of_ds, time_running(:, 3), '-o');
xlabel('Average size of data chunks (KB)');
ylabel('Running time (sec)');
legend('First opportunity', 'Balanced DOP', ...
    'Location', 'northwest');
saveas(gcf, 'fig/big_ds_size_2_time.fig');

figure;
plot(size_of_ds, rate_total(:, 1), ...
    size_of_ds, rate_total(:, 4), '-*'); %, ...
    %size_of_ds, rate_total(:, 7), '-o');
xlabel('Average size of data chunks (KB)');
ylabel('Portion of important data chunks uploaded');
legend('First opportunity', 'Balanced DOP', ...
    'Location', 'southwest');
saveas(gcf, 'fig_2/big_ds_size_2_high.fig');

figure;
plot(size_of_ds, rate_total(:, 2), ...
    size_of_ds, rate_total(:, 5), '-*'); %, ...
    %size_of_ds, rate_total(:, 8), '-o');
xlabel('Average size of data chunks (KB)');
ylabel('Portion of medium data chunks uploaded');
legend('First opportunity', 'Balanced DOP', ...
    'Location', 'southwest');
saveas(gcf, 'fig_2/big_ds_size_2_medium.fig');

figure;
plot(size_of_ds, rate_total(:, 3), ...
    size_of_ds, rate_total(:, 6), '-*'); %, ...
    %size_of_ds, rate_total(:, 9), '-o');
xlabel('Average size of data chunks (KB)');
ylabel('Portion of unimp. data chunks uploaded');
legend('First opportunity', 'Balanced DOP', ...
    'Location', 'southwest');
saveas(gcf, 'fig_2/big_ds_size_2_low.fig');

figure;
plot(size_of_ds, rate_all_total(:, 1), ...
    size_of_ds, rate_all_total(:, 2), '-*'); %, ...
    %size_of_ds, rate_all_total(:, 3), '-o');
xlabel('Average size of data chunks (KB)');
ylabel('Portion of data chunks uploaded');
legend('First opportunity', 'Balanced DOP', ...
    'Location', 'southwest');
saveas(gcf, 'fig_2/big_ds_size_2_all.fig');

figure;
plot(size_of_ds, length_task(:, 1), ...
    size_of_ds, length_task(:, 2), '-*'); %, ...
    %size_of_ds, length_task(:, 3), '-o');
xlabel('Average size of data chunks (KB)');
ylabel('Time to complete all data collection (sec)');
legend('First opportunity', 'Balanced DOP', ...
    'Location', 'northwest');
saveas(gcf, 'fig_2/big_ds_size_2_length.fig');

save('mat/big_ds_size_2.mat')
