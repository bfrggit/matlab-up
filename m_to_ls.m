function [ls] = m_to_ls(m)
%M_TO_LS        Convert a schedule matrix to a schedule list
%M_TO_LS(m)
%   m           Schedule matrix

ns = (1:size(m, 2))';
ls = m * ns;

end

