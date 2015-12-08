% Author: Charles ZHU
% --
% Statistics, w/ fixed path length, variable number of OP
% Config file generation script

init_p;

t_wait = 20;

% Initialize environment
clc;
rand('state', 0); %#ok<RAND>
randn('state', 0); %#ok<RAND>

% Constants for DS
N_DS = 30;
DX_MU = 180;
DX_SIGMA = 40;
R_0 = 1500;
S_M = 5000;
S_RANGE = 3000;
DD_M = 80;
DX_MIN_DS = 60;
D_OFFSET = 120;
DD_RANGE = 120;

% Constants for OP
LENGTH = 6000;
ER_MU = 500;
ER_SIGMA = 250;
ER_MIN = 25;
DX_MIN_OP = 200;

% Constants
N_LOOP = 10;

number_of_op = (3:3:15)';
dxs_m = LENGTH./ number_of_op;
nm_op = size(number_of_op, 1);
loop_n = N_LOOP * nm_op;
reward_total = zeros(nm_op, 3);
time_running = zeros(nm_op, 3);

mkdir('config');
mkdir('config', 'change_op_number');

tic
for j = 1:nm_op
    rw1_total = 0.0; rw2_total = 0.0; rw3_total = 0.0;
    et_plan1 = 0.0; et_plan2 = 0.0; et_plan3 = 0.0;
    
    mkdir('config/change_op_number', sprintf('%d', number_of_op(j)));
    
    for k = 1:N_LOOP
        mkdir(sprintf('config/change_op_number/%d', number_of_op(j)), ...
            sprintf('case_%d', k));
        
        % Generate demo instances
        v_ds = mk_vec_ds_new_min(N_DS, DX_MU, DX_SIGMA, R_0, ...
            S_M, S_RANGE, DD_M, ...
            DX_MIN_DS, ...
            D_OFFSET, DD_RANGE);
        v_op = mk_vec_op_min(number_of_op(j), dxs_m(j), ER_MU, ER_SIGMA, ER_MIN, DX_MIN_OP);
        
        % Write to scenario configuration file
        fh = fopen( ...
            sprintf('config/change_op_number/%d/case_%d/case.up.deployment', ...
                number_of_op(j), k), ...
            'w');
        fprintf(fh, 'MOBILE_DATA_COLLECTOR\n');
        fprintf(fh, '%d %d\n', V_MDC, R_0 * 8);
        fprintf(fh, '\n');
        fprintf(fh, 'ACCESS_POINTS\n');
        for it_ap = 1:number_of_op(j)
            fprintf(fh, '%d %d\n', v_op(it_ap, 1), v_op(it_ap, 2) * 8);
        end
        fprintf(fh, '\n');
        fprintf(fh, 'DATA_SITES\n');
        for it_ds = 1:N_DS
            fprintf(fh, ...
                '%d %d %d %.1f\n', ...
                v_ds(it_ds, 1), ...
                round(v_ds(it_ds, 3) * 1000 / 1024), ...
                max([v_ds(it_ds, 4) 1], [], 2), ...
                v_ds(it_ds, 5));
        end
        fprintf(fh, '\n');
        fclose(fh);
        
        loop_j = k + (j - 1)* N_LOOP;
        fprintf(sprintf('Running loop %d of %d...\n', loop_j, loop_n));
        
        % ASAP planning
        et = cputime;
        [mat_m, ls] = plan_asap(v_ds, v_op);
        et_plan1 = et_plan1 + (cputime - et);
        
        % Calculate actual upload time
        t_up = vec_t_up(v_ds, v_op, mat_m, t_wait);
        v_f = vec_f(v_ds, t_up);
        
        rw1 = reward(v_ds, v_f);
        rw1_total = rw1_total + rw1;

        % Write to plan specification file
        fh = fopen( ...
            sprintf('config/change_op_number/%d/case_%d/asap.plan', ...
                number_of_op(j), k), ...
            'w');
        fprintf(fh, '%d\n', N_DS);
        for it_ds = 1:N_DS
            fprintf(fh, '%d %d\n', it_ds, ls(it_ds));
        end
        fclose(fh);
		t_up_sorted = sortrows([(1:N_DS)' t_up], 2);
        dlmwrite( ...
            sprintf('config/change_op_number/%d/case_%d/asap.txt', ...
                number_of_op(j), k), ...
            t_up_sorted);
        
        % Algorithm 4 planning
        et = cputime;
        [mat_m, ls] = plan_alg4(v_ds, v_op, t_wait);
        et_plan2 = et_plan2 + (cputime - et);
        
        % Calculate actual upload time
        t_up = vec_t_up(v_ds, v_op, mat_m, t_wait);
        v_f = vec_f(v_ds, t_up);
        
        rw2 = reward(v_ds, v_f);
        rw2_total = rw2_total + rw2;
        
        % Write to plan specification file
        fh = fopen( ...
            sprintf('config/change_op_number/%d/case_%d/alg4.plan', ...
                number_of_op(j), k), ...
            'w');
        fprintf(fh, '%d\n', N_DS);
        for it_ds = 1:N_DS
            fprintf(fh, '%d %d\n', it_ds, ls(it_ds));
        end
        fclose(fh);
		t_up_sorted = sortrows([(1:N_DS)' t_up], 2);
        dlmwrite( ...
            sprintf('config/change_op_number/%d/case_%d/alg4.txt', ...
                number_of_op(j), k), ...
            t_up_sorted);
        
        % ASAP planning
        [cst_m, cst_ls] = plan_asap(v_ds, v_op);

        % GA planning
        et = cputime;
        [mat_m, ls] = plan_ga(v_ds, v_op, cst_ls, t_wait);
        et_plan3 = et_plan3 + (cputime - et);
        fprintf('\n');
        
        % Calculate actual upload time
        t_up = vec_t_up(v_ds, v_op, mat_m, t_wait);
        v_f = vec_f(v_ds, t_up);
        
        rw3 = reward(v_ds, v_f);
        rw3_total = rw3_total + rw3;

        % Write to plan specification file
        fh = fopen( ...
            sprintf('config/change_op_number/%d/case_%d/ga.plan', ...
                number_of_op(j), k), ...
            'w');
        fprintf(fh, '%d\n', N_DS);
        for it_ds = 1:N_DS
            fprintf(fh, '%d %d\n', it_ds, ls(it_ds));
        end
        fclose(fh);
		t_up_sorted = sortrows([(1:N_DS)' t_up], 2);
        dlmwrite( ...
            sprintf('config/change_op_number/%d/case_%d/ga.txt', ...
                number_of_op(j), k), ...
            t_up_sorted);
    end
    reward_total(j, 1) = rw1_total / N_LOOP;
    reward_total(j, 2) = rw2_total / N_LOOP;
    reward_total(j, 3) = rw3_total / N_LOOP;
    time_running(j, 1) = et_plan1 / N_LOOP;
    time_running(j, 2) = et_plan2 / N_LOOP;
    time_running(j, 3) = et_plan3 / N_LOOP;
end
toc
plot(number_of_op, reward_total(:, 1), ...
    number_of_op, reward_total(:, 2), '-*', ...
    number_of_op, reward_total(:, 3), '-o');
xlabel('Number of upload opportunities');
ylabel('Weighted overall utility');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig/conf_op_number_reward.fig');

figure;
plot(number_of_op, time_running(:, 1), ...
    number_of_op, time_running(:, 2), '-*', ...
    number_of_op, time_running(:, 3), '-o');
xlabel('Number of upload opportunities');
ylabel('Running time (sec)');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig/conf_op_number_time.fig');

save('mat/conf_op_number.mat')
