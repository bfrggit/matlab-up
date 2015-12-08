function [ret] = mk_vec_ds_new_min(n_ds, dx_mu, dx_sigma, r_0, s_mid, s_range, dd_mid, dx_min, d_offset, dd_range)
%MK_VEC_DS      Make a vector of DS
%MK_VEC_DS(n_ds, dx_mu, dx_sigma, r_0, s_0, dd_mid)
%   n_ds        Number of DS
%   dx_mu       Mean of distances between DS ~ Normal dist.
%   dx_sigma    STDEV of distances between DS
%   r_0         Pre-determined downloading rate
%   s_mid       Mean of chunk sizes ~ Uniform dist.
%   s_range     Half range of chunk sizes
%   dd_mid      Mean of deadline differences between DS ~ Uniform dist.
%   dx_min      Minimum distance between DS
%   d_offset    Offset of deadlines
%   dd_range    Half range of deadline differences

if nargin < 10
    dd_range = dd_mid;
end

if nargin < 9
    d_offset = 0;
end

global P_DIST;

p_cus = cumsum(P_DIST(:, 2));
p_val = P_DIST(:, 1);

dx = max(round(normrnd(dx_mu, dx_sigma, n_ds, 1)), dx_min);
x = cumsum(dx);
r = repmat(r_0, n_ds, 1);
s = round(rand(n_ds, 1) * s_range * 2 + s_mid - s_range);
dd = round(rand(n_ds, 1) * dd_range * 2 + dd_mid - dd_range);
d = cumsum(dd) + d_offset;
p = zeros(n_ds, 1);
for j = 1:n_ds
    p(j) = p_val(sum(p_cus <= rand()) + 1);
end
ret = [x r s d p];

end
