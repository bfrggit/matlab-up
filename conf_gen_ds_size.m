% Author: Charles ZHU
% --
% Statistics, w/ fixed number of DS, variable size of DS
% Config file generation script

init_p;

t_wait = 15;

% Initialize environment
clc;
rand('state', 0); %#ok<RAND>
randn('state', 0); %#ok<RAND>

% Constants for DS
N_DS = 30;
DX_MU = 180;
DX_SIGMA = 40;
R_0 = 1500;
%S_M = 5000;
%S_RANGE = 3000;
DD_M = 80;
DX_MIN_DS = 60;
D_OFFSET = 120;
DD_RANGE = 120;

% Constants for OP
N_OP = 12;
DX_M = 500;
ER_MU = 500;
ER_SIGMA = 250;
ER_MIN = 25;
DX_MIN_OP = 200;

% Constants
N_LOOP = 10;

size_of_ds = (1000:2000:9000)';
ss_range = size_of_ds * 0.6;
nm_ds = size(size_of_ds, 1);
loop_n = N_LOOP * nm_ds;
reward_total = zeros(nm_ds, 3);
time_running = zeros(nm_ds, 3);

mkdir('config');
mkdir('config', 'change_ds_size');

tic
for j = 1:nm_ds
    rw1_total = 0.0; rw2_total = 0.0; rw3_total = 0.0;
    et_plan1 = 0.0; et_plan2 = 0.0; et_plan3 = 0.0;
    
    mkdir('config/change_ds_size', sprintf('%d', size_of_ds(j)));
    
    for k = 1:N_LOOP
        mkdir(sprintf('config/change_ds_size/%d', size_of_ds(j)), ...
            sprintf('case_%d', k));
        
        % Generate demo instances
        v_ds = mk_vec_ds_new_min(N_DS, DX_MU, DX_SIGMA, R_0, ...
            size_of_ds(j), ss_range(j), DD_M, ...
            DX_MIN_DS, ...
            D_OFFSET, DD_RANGE);
        v_op = mk_vec_op_min(N_OP, DX_M, ER_MU, ER_SIGMA, ER_MIN, DX_MIN_OP);
        
        % Write to scenario configuration file
        fh = fopen( ...
            sprintf('config/change_ds_size/%d/case_%d/case.up.deployment', ...
                size_of_ds(j), k), ...
            'w');
        fprintf(fh, 'MOBILE_DATA_COLLECTOR\n');
        fprintf(fh, '%d %d\n', V_MDC, R_0 * 8);
        fprintf(fh, '\n');
        fprintf(fh, 'ACCESS_POINTS\n');
        for it_ap = 1:N_OP
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
            sprintf('config/change_ds_size/%d/case_%d/asap.plan', ...
                size_of_ds(j), k), ...
            'w');
        fprintf(fh, '%d\n', N_DS);
        for it_ds = 1:N_DS
            fprintf(fh, '%d %d\n', it_ds, ls(it_ds));
        end
        fclose(fh);
		t_up_sorted = sortrows([(1:N_DS)' t_up], 2);
        dlmwrite( ...
            sprintf('config/change_ds_size/%d/case_%d/asap.txt', ...
                size_of_ds(j), k), ...
            t_up_sorted);
        
        % Algorithm 4 planning
        et = cputime;
        [mat_m, ls] = plan_alg4x(v_ds, v_op, t_wait);
        et_plan2 = et_plan2 + (cputime - et);
        
        % Calculate actual upload time
        t_up = vec_t_up(v_ds, v_op, mat_m, t_wait);
        v_f = vec_f(v_ds, t_up);
        
        rw2 = reward(v_ds, v_f);
        rw2_total = rw2_total + rw2;
        
        % Write to plan specification file
        fh = fopen( ...
            sprintf('config/change_ds_size/%d/case_%d/alg4.plan', ...
                size_of_ds(j), k), ...
            'w');
        fprintf(fh, '%d\n', N_DS);
        for it_ds = 1:N_DS
            fprintf(fh, '%d %d\n', it_ds, ls(it_ds));
        end
        fclose(fh);
		t_up_sorted = sortrows([(1:N_DS)' t_up], 2);
        dlmwrite( ...
            sprintf('config/change_ds_size/%d/case_%d/alg4.txt', ...
                size_of_ds(j), k), ...
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
            sprintf('config/change_ds_size/%d/case_%d/ga.plan', ...
                size_of_ds(j), k), ...
            'w');
        fprintf(fh, '%d\n', N_DS);
        for it_ds = 1:N_DS
            fprintf(fh, '%d %d\n', it_ds, ls(it_ds));
        end
        fclose(fh);
		t_up_sorted = sortrows([(1:N_DS)' t_up], 2);
        dlmwrite( ...
            sprintf('config/change_ds_size/%d/case_%d/ga.txt', ...
                size_of_ds(j), k), ...
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
plot(size_of_ds, reward_total(:, 1), ...
    size_of_ds, reward_total(:, 2), '-*', ...
    size_of_ds, reward_total(:, 3), '-o');
xlabel('Size of one single data chunk (kB)');
ylabel('Weighted overall utility');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig/conf_ds_size_reward.fig');

figure;
plot(size_of_ds, time_running(:, 1), ...
    size_of_ds, time_running(:, 2), '-*', ...
    size_of_ds, time_running(:, 3), '-o');
xlabel('Size of one single data chunk (kB)');
ylabel('Running time (sec)');
legend('First opportunity', 'Proposed algorithm', 'Genetic algorithm');
saveas(gcf, 'fig/conf_ds_size_time.fig');

save('mat/conf_ds_size.mat')
