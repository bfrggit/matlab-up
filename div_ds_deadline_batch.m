% Author: Charles ZHU
% --
% Dividing DS into only two OP

init_p;

% Initialize environment
clc;
rand('state', 0); %#ok<RAND>
randn('state', 0); %#ok<RAND>

% Constants for DS
N_DS = 100;
X_0 = 1000;
R_0 = 1500;
S_0 = 5000;
p_cus = cumsum(P_DIST(:, 2));
p_val = P_DIST(:, 1);

% Constants for OP
V_OP = [2500 50; 3500 500];

% Constants
N_LOOP = 1;

deadline_offset_of_ds = (3100:100:5000)';
nm_ds = size(deadline_offset_of_ds, 1);
loop_n = N_LOOP * nm_ds;
reward_total = zeros(nm_ds, 3);
time_running = zeros(nm_ds, 3);
var_u_total = zeros(nm_ds, 3);

tic
for j = 1:nm_ds
    rw1_total = 0.0;
    rw2_total = 0.0;
    rw3_total = 0.0;
    et_plan1 = 0.0;
    et_plan2 = 0.0;
    et_plan3 = 0.0;
    u1_total = 0.0;
    u2_total = 0.0;
    u3_total = 0.0;
    for k = 1:N_LOOP
        % Generate demo instances
        v_ds = repmat([X_0 R_0 S_0 deadline_offset_of_ds(j) 0], N_DS, 1);
        v_op = V_OP;
        for x = 1:N_DS
            v_ds(x, 5) = p_val(sum(p_cus <= rand()) + 1);
        end
        
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
        u1 = sum(ls == 1);
        u1_total = u1_total + u1;
        
        % Algorithm 4 planning
        et = cputime;
        [mat_m, ls] = plan_alg4(v_ds, v_op, T_WAIT);
        et_plan2 = et_plan2 + (cputime - et);
        
        % Calculate actual upload time
        t_up = vec_t_up(v_ds, v_op, mat_m, T_WAIT);
        v_f = vec_f(v_ds, t_up);
        
        rw2 = reward(v_ds, v_f);
        rw2_total = rw2_total + rw2;
        u2 = sum(ls == 1);
        u2_total = u2_total + u2;
        
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
        u3 = sum(ls == 1);
        u3_total = u3_total + u3;
    end
    reward_total(j, 1) = rw1_total / N_LOOP;
    reward_total(j, 2) = rw2_total / N_LOOP;
    reward_total(j, 3) = rw3_total / N_LOOP;
    time_running(j, 1) = et_plan1 / N_LOOP;
    time_running(j, 2) = et_plan2 / N_LOOP;
    time_running(j, 3) = et_plan3 / N_LOOP;
    var_u_total(j, 1) = u1 / N_LOOP;
    var_u_total(j, 2) = u2 / N_LOOP;
    var_u_total(j, 3) = u3 / N_LOOP;
end
toc
plot(deadline_offset_of_ds, reward_total(:, 1), deadline_offset_of_ds, reward_total(:, 2), '-*', deadline_offset_of_ds, reward_total(:, 3), '-o');
xlabel('Deadline offset of data sites');
ylabel('Weighted overall utility');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig/div_ds_deadline_reward.fig');

figure;
plot(deadline_offset_of_ds, time_running(:, 1), deadline_offset_of_ds, time_running(:, 2), '-*', deadline_offset_of_ds, time_running(:, 3), '-o');
xlabel('Deadline offset of data sites');
ylabel('Running time (sec)');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig/div_ds_deadline_time.fig');

figure;
plot(deadline_offset_of_ds, var_u_total(:, 1), deadline_offset_of_ds, var_u_total(:, 2), '-*', deadline_offset_of_ds, var_u_total(:, 3), '-o');
xlabel('Deadline offset of data sites');
ylabel('Number of data chunks planned at first opportunity');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig/div_ds_deadline_var_u.fig');

save('mat/div_ds_deadline.mat')
