function [ret] = mk_vec_ds_min(n_ds, dx_mu, dx_sigma, r_0, s_0, dd_mid, dx_min)
%MK_VEC_DS_MIN  Make a vector of DS with minimum distance
%MK_VEC_DS_MIN(n_ds, dx_mu, dx_sigma, r_0, s_0, dd_mid, dx_min)
%   n_ds        Number of DS
%   dx_mu       Mean of distances between DS ~ Normal dist.
%   dx_sigma    STDEV of distances between DS
%   r_0         Pre-determined downloading rate
%   s_0         Pre-determined chunk size
%   dd_mid      Mean of deadline differences between DS ~ Uniform dist.
%   dx_min      Minimum distance between DS

global P_DIST;

p_cus = cumsum(P_DIST(:, 2));
p_val = P_DIST(:, 1);

dx = min(round(normrnd(dx_mu, dx_sigma, n_ds, 1)), dx_min);
x = cumsum(dx);
r = repmat(r_0, n_ds, 1);
s = repmat(s_0, n_ds, 1);
dd = round(rand(n_ds, 1) * dd_mid * 2);
d = cumsum(dd);
p = zeros(n_ds, 1);
for j = 1:n_ds
    p(j) = p_val(sum(p_cus <= rand()) + 1);
end
ret = [x r s d p];

end
