% Author: Charles ZHU
% --
% Statistics, w/ fixed OP, variable sigma of estimated rate of OP
% Batch script (big version)

init_p;

% Initialize environment
clc;
rand('state', 0); %#ok<RAND>
randn('state', 0); %#ok<RAND>

% Constants for DS
N_DS = 1000;
DX_MU = 180;
DX_SIGMA = 60;
R_0 = 1500;
S_M = 5000;
S_RANGE = 3000;
DD_M = 80;
D_OFFSET = 120;
DD_RANGE = 120;

% Constants for OP
N_OP = 40;
DX_M = 5000;
ER_MU = 500;
ER_SIGMA = 250;
ER_MIN = 25;

% Constants
N_LOOP = 10;

sigma_rate_of_op = (0:15:285)';
nm_op = size(sigma_rate_of_op, 1);
loop_n = N_LOOP * nm_op;
reward_total = zeros(nm_op, 3);
time_running = zeros(nm_op, 3);
rate_total = zeros(nm_op, 9);
rate_all_total = zeros(nm_op, 3);
length_task = zeros(nm_op, 3);

tic
for j = 1:nm_op
    reward_acc = zeros(1, 3);
    time_acc = zeros(1, 3);
    rate_acc = zeros(3, 6);
    length_acc = zeros(1, 3);
    
    for k = 1:N_LOOP
        % Generate demo instances
        v_ds = mk_vec_ds_new(N_DS, DX_MU, DX_SIGMA, R_0, S_M, S_RANGE, DD_M, D_OFFSET, DD_RANGE);
        v_op = mk_vec_op(N_OP, DX_M, ER_MU, sigma_rate_of_op(j), ER_MIN);
        
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
plot(sigma_rate_of_op, reward_total(:, 1), ...
    sigma_rate_of_op, reward_total(:, 2), '-*'); %, ...
    %sigma_rate_of_op, reward_total(:, 3), '-o');
xlabel('Standard deviation of bandwidth of upload opportunities (kB/s)');
ylabel('Weighted overall utility');
legend('First opportunity', 'Proposed algorithm');
saveas(gcf, 'fig/big_op_sigma_reward.fig');

figure;
plot(sigma_rate_of_op, time_running(:, 1), ...
    sigma_rate_of_op, time_running(:, 2), '-*'); %, ...
    %sigma_rate_of_op, time_running(:, 3), '-o');
xlabel('Standard deviation of bandwidth of upload opportunities (kB/s)');
ylabel('Running time (sec)');
legend('First opportunity', 'Proposed algorithm');
saveas(gcf, 'fig/big_op_sigma_time.fig');

figure;
plot(sigma_rate_of_op, rate_total(:, 1), ...
    sigma_rate_of_op, rate_total(:, 4), '-*'); %, ...
    %sigma_rate_of_op, rate_total(:, 7), '-o');
xlabel('Standard deviation of bandwidth of upload opportunities (kB/s)');
ylabel('Portion of high priority data chunks uploaded');
legend('First opportunity', 'Proposed algorithm');
saveas(gcf, 'fig_2/big_op_sigma_high.fig');

figure;
plot(sigma_rate_of_op, rate_total(:, 2), ...
    sigma_rate_of_op, rate_total(:, 5), '-*'); %, ...
    %sigma_rate_of_op, rate_total(:, 8), '-o');
xlabel('Standard deviation of bandwidth of upload opportunities (kB/s)');
ylabel('Portion of medium priority data chunks uploaded');
legend('First opportunity', 'Proposed algorithm');
saveas(gcf, 'fig_2/big_op_sigma_medium.fig');

figure;
plot(sigma_rate_of_op, rate_total(:, 3), ...
    sigma_rate_of_op, rate_total(:, 6), '-*'); %, ...
    %sigma_rate_of_op, rate_total(:, 9), '-o');
xlabel('Standard deviation of bandwidth of upload opportunities (kB/s)');
ylabel('Portion of low priority data chunks uploaded');
legend('First opportunity', 'Proposed algorithm');
saveas(gcf, 'fig_2/big_op_sigma_low.fig');

figure;
plot(sigma_rate_of_op, rate_all_total(:, 1), ...
    sigma_rate_of_op, rate_all_total(:, 2), '-*'); %, ...
    %sigma_rate_of_op, rate_all_total(:, 3), '-o');
xlabel('Standard deviation of bandwidth of upload opportunities (kB/s)');
ylabel('Portion of data chunks uploaded');
legend('First opportunity', 'Proposed algorithm');
saveas(gcf, 'fig_2/big_op_sigma_all.fig');

figure;
plot(sigma_rate_of_op, length_task(:, 1), ...
    sigma_rate_of_op, length_task(:, 2), '-*'); %, ...
    %sigma_rate_of_op, length_task(:, 3), '-o');
xlabel('Standard deviation of bandwidth of upload opportunities (kB/s)');
ylabel('Total time to finish all data collection (sec)');
legend('First opportunity', 'Proposed algorithm');
saveas(gcf, 'fig_2/big_op_sigma_length.fig');

save('mat/big_op_sigma.mat')
