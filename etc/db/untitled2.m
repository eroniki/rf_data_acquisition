%% Individual Path Loss Exponent
close all;
n_test = 1:20;
max_n = max(n_test);
e_bar = [];
n_tested = [];
for n=n_test
    n_tested = [n_tested, n];
    e = [];
    for i=1:1543
        test_point = grid_labels(i,:);
        pl_vector = pt - data_lora(i,:);
        d_hat = ldpl(dist, pl_vector', pl_at_center, n, std_lora);
        d = pdist2(pos_node,test_point);
        e = [e, loss(d_hat,d)];
    end
    buff = mean(e, 2, 'omitnan');
    e_bar = [e_bar, buff];
    disp(['n: ', num2str(n), ' e_bar: ', num2str(buff')]);
    figure(1);
    plot(n_tested, e_bar(1,:), n_tested, e_bar(2,:), n_tested, e_bar(3,:), ...
        n_tested, e_bar(4, :), n_tested, e_bar(5, :), n_tested, e_bar(6,:), ...
        n_tested, e_bar(7, :), n_tested, e_bar(8, :));
    grid on;
    xlim([min(n_test), max(n_test)]);
    legend('0', '1', '2', '3', '4', '5', '6', '7');
    xlabel('Path Loss Exponent (n)');
    ylabel('Loss');
    drawnow;
end
% size(e_bar)
% [min_val, min_id] = min(error_map);
% plot(n_test, e_bar); grid on;
% 
[min_val, min_id] = min(e_bar, [], 2);
path_loss_exp_ind = n_test(min_id);
disp('Path Loss Exponents:');
disp(vpa(path_loss_exp_ind, 8));
disp('Corresponding Loss Values:');
disp(min_val);