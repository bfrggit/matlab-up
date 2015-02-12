function [ret] = mat_s_dd(mat_m)
%MAT_S_DD       Calculate matrix S_DD (DS at same OP with DS)
%MAT_S_DD(mat_m)
%   mat_m       Plan matrix m

% Right bracket is similar to m_to_s (can be replaced)
ret = mat_m(:, (1:size(mat_m, 2))*mat_m');

end

