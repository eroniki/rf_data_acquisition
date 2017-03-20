function [d_hat] = ldpl(d0, pl_d, pl_d0, n, std)
    rnd_= normrnd(zeros([numel(pl_d), 1]), std');
    d_hat = 10.^((pl_d - pl_d0 - rnd_)./(10*n)).*d0;
end

