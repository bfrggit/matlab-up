function [m, ls] = plan_alg4(v_ds, v_op)
%PLAN_ALG4      Generate a plan using algorithm 4
%PLAN_ALG4(v_ds, v_op)
%   v_ds        DS vector
%   v_op        OP vector

n_ds = size(v_ds, 1);
n_op = size(v_op, 1);

% Find bounds
b_do = mat_b_od(v_op, v_ds)';
b_shift = [zeros(size(b_do, 1), 1) b_do(:, 1:size(b_do, 2) - 1)];
cst_ls = m_to_ls(b_do - b_shift);
lb = cst_ls';
ub = repmat(n_op, 1, n_ds);

% Create a dynamic list to keep unallocated DS
sort_ds_dl = sortrows([(1:n_ds)' v_ds(:, 4)], 2);
%sort_ds = sort_ds_dl(:, 1);
dynamic_ds = cell(1, n_ds);
for j = 1:n_ds
    dynamic_ds{j} = sort_ds_dl(n_ds - j + 1, 1);
end
first_op = lb(1); % Maintain earliest available OP
present_x = ub'; % Maintain present schedule vector

while size(dynamic_ds, 2) > 0
    sacrifice = 0;
    sacrificed_ds = [];
    while 1
        % Obtain working DS
        nd = size(dynamic_ds, 2);
        present_ds = dynamic_ds{nd};  % Present working DS
        dynamic_ds(nd) = [];          % Remove DS from dynamic list
        if lb(present_ds) > first_op
            first_op = lb(present_ds);
        end
        %fprintf('\nIn loop for %d as current DS...\n', present_ds);

        % Fastest OP that ensures in-time delivery of DS
        chosen_op = n_op; % Chosen OP
        for j = first_op:n_op % Test OP
            tmp_x = present_x;
            tmp_x(present_ds) = j;
            v_f = vec_f(v_ds, vec_t_up(v_ds, v_op, ls_to_m(tmp_x, n_op)));
            if v_f(present_ds) > 1-1e-6 % In-time delivery
                if chosen_op >= n_op || v_op(j, 2) > v_op(chosen_op, 2)
                    chosen_op = j;
                end
            else
                break
            end
        end
        v_f = vec_f(v_ds, vec_t_up(v_ds, v_op, ls_to_m(present_x, n_op)));

        if chosen_op < n_op % If this DS does fit
            present_x(present_ds) = chosen_op;
            
            %fprintf('Chosen OP %d for DS %d.\n', chosen_op, present_ds);
            %fprintf('DS %d is scheduled at OP %d.\n', present_ds, chosen_op);

            % Dump sacrificed DS
            for j = 1:size(sacrificed_ds, 1)
                single_ds = sacrificed_ds(j, :);
                dynamic_ds = [dynamic_ds single_ds(1)]; %#ok<AGROW>
                %fprintf('Need to reschedule sacrificed OP %d.\n', single_ds(1));
            end
            break
        else % If this DS does not fit
            %fprintf('Unable to choose OP for DS %d.\n', present_ds);
            
            % Choose one DS to sacrifice in order to fit the present
            chosen_ds = 0;
            for j = 1:n_ds
                if j ~= present_ds && present_x(j) <= first_op ...
                        && present_x(j) < n_op
                    % Criteria: lower priority / later deadline
                    if v_ds(j, 5) < v_ds(present_ds, 5) ...
                            || v_ds(j, 5) == v_ds(present_ds, 5) ...
                            && v_ds(j, 4) > v_ds(present_ds, 4)
                        if chosen_ds <= 0 ...
                                || v_ds(j, 5) < v_ds(chosen_ds, 5) ...
                                || v_ds(j, 5) == v_ds(chosen_ds, 5) ...
                                && v_ds(j, 4) > v_ds(chosen_ds, 4)
                            chosen_ds = j;
                        end
                    end
                end
            end
            
            if chosen_ds > 0
                %fprintf('Chosen DS %d to sacrifice.\n', chosen_ds);
                %fprintf('DS %d is %d, %f to sacrifice for DS %d which is %d, %f.\n', ...
                    %chosen_ds, ...
                    %v_ds(chosen_ds, 4), ...
                    %v_ds(chosen_ds, 5), ...
                    %present_ds, ...
                    %v_ds(present_ds, 4), ...
                    %v_ds(present_ds, 5));
                sacrifice = sacrifice + v_f(chosen_ds) * v_ds(chosen_ds, 5);
            end
            if chosen_ds <= 0 || sacrifice >= v_ds(present_ds, 5) % Too much sacrifice
                %if chosen_ds > 0
                    %fprintf('Too much sacrifice for DS %d.\n', present_ds);
                %else
                    %fprintf('Unable to choose DS to sacrifice for DS %d.\n', present_ds);
                %end
                chosen_new_op = first_op;
                for j = first_op:(n_op-1)
                    if v_op(j, 2) > v_op(chosen_new_op, 2)
                        chosen_new_op = j;
                    end
                end
                present_x(present_ds) = chosen_new_op;
                %fprintf('DS %d is scheduled at OP %d.\n', present_ds, chosen_new_op);

                % Recover sacrificed DS
                for j = 1:size(sacrificed_ds, 1)
                    single_ds = sacrificed_ds(j, :);
                    present_x(single_ds(1)) = single_ds(2);
                    %fprintf('DS %d is recovered at OP %d.\n', single_ds(1), single_ds(2));
                end
                break
            else
                sacrificed_ds = [sacrificed_ds; chosen_ds present_x(chosen_ds)]; %#ok<AGROW>
                present_x(chosen_ds) = n_op;
                dynamic_ds = [dynamic_ds present_ds]; %#ok<AGROW>
                %fprintf('Sacrificed DS %d for DS %d.\n', chosen_ds, present_ds);
            end
        end
    end
end

ls = present_x;
m = ls_to_m(ls, n_op);

end
