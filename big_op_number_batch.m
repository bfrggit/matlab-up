% Author: Charles ZHU
% --
% Statistics, w/ fixed path length, variable number of OP
% Batch script (big version)

init_p;

% Initialize environment
clc;
%rand('state', 0); %#ok<RAND>
%randn('state', 0); %#ok<RAND>
rng('default');
rng(0);

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
LENGTH = 200000;
ER_MU = 500;
ER_SIGMA = 250;
ER_MIN = 25;

% Constants
N_LOOP = 10;

% Random seeds for loops
rng_seeds = randi(2 ^ 32 - 1, N_LOOP, 2);

number_of_op = (10:10:200)';
dxs_m = LENGTH./ number_of_op;
nm_op = size(number_of_op, 1);
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
        rng(rng_seeds(k, 1));
        v_ds = mk_vec_ds_new(N_DS, DX_MU, DX_SIGMA, R_0, ...
        	S_M, S_RANGE, DD_M, D_OFFSET, DD_RANGE);
        rng(rng_seeds(k, 2));
        v_op = mk_vec_op(number_of_op(j), dxs_m(j), ER_MU, ER_SIGMA, ER_MIN);
        
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
plot(number_of_op, reward_total(:, 1), ...
    number_of_op, reward_total(:, 2), '-*'); %, ...
    %number_of_op, reward_total(:, 3), '-o');
xlabel('Number of upload opportunities');
ylabel('Weighted overall utility');
legend('First opportunity', 'Balanced DOP', ...
	'Location', 'southeast');
saveas(gcf, 'fig/big_op_number_reward.fig');

figure;
plot(number_of_op, time_running(:, 1), ...
    number_of_op, time_running(:, 2), '-*'); %, ...
    %number_of_op, time_running(:, 3), '-o');
xlabel('Number of upload opportunities');
ylabel('Running time (sec)');
legend('First opportunity', 'Balanced DOP');
saveas(gcf, 'fig/big_op_number_time.fig');

figure;
plot(number_of_op, rate_total(:, 1), ...
    number_of_op, rate_total(:, 4), '-*'); %, ...
    %number_of_op, rate_total(:, 7), '-o');
xlabel('Number of upload opportunities');
ylabel('Portion of important data chunks uploaded');
legend('First opportunity', 'Balanced DOP', ...
	'Location', 'southeast');
saveas(gcf, 'fig_2/big_op_number_high.fig');

figure;
plot(number_of_op, rate_total(:, 2), ...
    number_of_op, rate_total(:, 5), '-*'); %, ...
    %number_of_op, rate_total(:, 8), '-o');
xlabel('Number of upload opportunities');
ylabel('Portion of medium data chunks uploaded');
legend('First opportunity', 'Balanced DOP', ...
	'Location', 'southeast');
saveas(gcf, 'fig_2/big_op_number_medium.fig');

figure;
plot(number_of_op, rate_total(:, 3), ...
    number_of_op, rate_total(:, 6), '-*'); %, ...
    %number_of_op, rate_total(:, 9), '-o');
xlabel('Number of upload opportunities');
ylabel('Portion of unimp. data chunks uploaded');
legend('First opportunity', 'Balanced DOP', ...
	'Location', 'southeast');
saveas(gcf, 'fig_2/big_op_number_low.fig');

figure;
plot(number_of_op, rate_all_total(:, 1), ...
    number_of_op, rate_all_total(:, 2), '-*'); %, ...
    %number_of_op, rate_all_total(:, 3), '-o');
xlabel('Number of upload opportunities');
ylabel('Portion of data chunks uploaded');
legend('First opportunity', 'Balanced DOP', ...
	'Location', 'southeast');
saveas(gcf, 'fig_2/big_op_number_all.fig');

figure;
plot(number_of_op, length_task(:, 1), ...
    number_of_op, length_task(:, 2), '-*'); %, ...
    %number_of_op, length_task(:, 3), '-o');
xlabel('Number of upload opportunities');
ylabel('Time to complete all data collection (sec)');
legend('First opportunity', 'Balanced DOP', ...
    'Location', 'northwest');
saveas(gcf, 'fig_2/big_op_number_length.fig');

save('mat/big_op_number.mat')
