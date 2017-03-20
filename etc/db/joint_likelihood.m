clc; clear all; close all;

x = 0:0.01:20;
mu = randi(min(17,max(x)+3), [10, 1]);
g = zeros(numel(mu), numel(x));
% disp(sort(mu));
for i=1:numel(mu)
    buff = normpdf(x, mu(i));
    g(i,:) = buff / sum(buff);
end
joint = prod(g)/sum(prod(g));
plot(x, joint, x, g); grid on; grid minor;
xlim([min(x), max(x)]);

mu_found = mean(g, 2);
mean_loc = [];

for i=1:10
    [val, idx] = max(g(i,:));
    [x(idx), mu_found(i), mu(i)];
    mean_loc = [mean_loc, idx];
end

[mu, x(mean_loc)']

[val, id] = max(joint);
x(id)