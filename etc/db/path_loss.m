close all;
verbose = 0;
%% Function Handles %
loss2 = @(d, d_hat, n, n_test_max) (d-d_hat).^2 + n/n_test_max;
loss = @(d, d_hat, n, n_test_max) (d-d_hat).^2;
localization_error = @(d, d_hat) sqrt(sum((d - d_hat).^2,2));
z_score = @(x) (x-mean(x,'omitnan')) ./ std(x, 'omitnan');
circle = @(x,y,r,ang) deal(x+r*cos(ang), y+r*sin(ang));
%%
center = [4, 13];
pt = 24;
lora_propagation = propagation_maps(:,:,17:24);
lora_propagation(~lora_propagation) = NaN;
pr_at_center = propagation_maps(13,4,17:24);
pl = zeros(25,7,8);
pr_at_center = pr_at_center(:);

dist = pdist2(pos_node,center);

pl = pt - lora_propagation;
pl_at_center = pt - pr_at_center;
%% Collective Path Loss Exponent
n_test = 1:0.02:8;
e_bar = [];
n_tested = [];
for n=n_test
    n_tested = [n_tested, n];
    loss_map = zeros(25,7,8);
    for i=1:25
        for j=1:7               
            test_point = [i,j];
            pl_vector = pl(test_point(1), test_point(2), :);
            pl_vector = pl_vector(:);
            d_hat = ldpl(dist, pl_vector, pl_at_center, n, std_lora);
            d = pdist2(pos_node, test_point);
            e =  loss(d_hat,d, n, max(n_test));
            loss_map(i,j,:) = e;
        end
    end
    buff = mean(loss_map(:), 'omitnan');
    e_bar = [e_bar, buff];
    if(verbose)
        disp(['n: ', num2str(n), ' e_bar: ', num2str(buff), num2str(size(e_bar))]);
    end
    figure(1);
    plot(n_tested, e_bar); grid on; grid minor;
    xlim([min(n_test), max(n_test)]);
    xlabel('Path Loss Exponent (n)');
    ylabel('Loss');
    drawnow;
end
[min_val, min_id] = min(e_bar);
path_loss_exp = n_test(min_id);
disp('Path Loss Exponent:');
disp(vpa(path_loss_exp, 8));
%% Ind. Path Loss Exponent
n_test = 1:0.2:8;
e_bar = [];
n_tested = [];
for n=n_test
    n_tested = [n_tested, n];
    loss_map = zeros(25,7,8);
    e = [];
    for i=1:25
        for j=1:7               
            test_point = [i,j];
            pl_vector = pl(test_point(1), test_point(2), :);
            pl_vector = pl_vector(:);
            d_hat = ldpl(dist, pl_vector, pl_at_center, n, std_lora);
            d = pdist2(pos_node, test_point);
            e =  [e, loss(d_hat,d, n, max(n_test))];
%             loss_map(i,j,:) = e;
        end
    end
    buff = mean(e, 2, 'omitnan');
    e_bar = [e_bar, buff];
    if(verbose)
        disp(['n: ', num2str(n), ' e_bar: ', num2str(buff'), num2str(size(e_bar))]);
    end
    figure(1);
    plot(n_tested, e_bar(1,:), n_tested, e_bar(2,:), n_tested, e_bar(3,:), ...
        n_tested, e_bar(4, :), n_tested, e_bar(5, :), n_tested, e_bar(6,:), ...
        n_tested, e_bar(7, :), n_tested, e_bar(8, :)); grid on; grid minor;
    legend('0', '1', '2', '3', '4', '5', '6', '7');
    xlim([min(n_test), max(n_test)]);
    xlabel('Path Loss Exponent (n)');
    ylabel('Loss');
    drawnow;
end
[min_val, min_id] = min(e_bar,[],2);
path_loss_exp_ind = n_test(min_id)';
disp('Path Loss Exponent:');
disp(vpa(path_loss_exp_ind, 8));
%% Re-evaluate the localization performance with the optimal ple
%% Grid-Localization with LDPL
close all;
error_map = zeros(25,7,8);
error_map_ind = zeros(25,7,8);
for i=1:25
    for j=1:7               
        test_point = [i,j];
        pl_vector = pl(test_point(1), test_point(2), :);
        pl_vector = pl_vector(:);

        d_hat = ldpl(dist, pl_vector, pl_at_center, path_loss_exp, std_lora);
        d_hat_ind = ldpl(dist, pl_vector, pl_at_center, path_loss_exp_ind, std_lora);
        d = pdist2(pos_node,test_point);
        e = localization_error(d_hat, d);
        e_ind = localization_error(d_hat_ind, d);
        error_map(i,j,:) = e;
        error_map_ind(i,j,:) = e_ind;
    end
end
%% Least-squares Sense Localization
close all;
e_vec = [];
e_vec_ind = [];
for i=1:25
    for j=1:7
    test_point = [i,j];
    pl_vector = pl(test_point(1), test_point(2), :);
    pl_vector = pl_vector(:);
    d_hat = ldpl(dist, pl_vector, pl_at_center, path_loss_exp, std_lora);
    d_hat_ind = ldpl(dist, pl_vector, pl_at_center, path_loss_exp_ind, std_lora);
    for k=1:8
        [x,y] = circle(pos_node(k,1), pos_node(k,2), d_hat(k), 0:0.001:2*pi);
        figure(1); plot(x,y); grid on; hold on;
        xlim([0,8]);
        ylim([0,25]);
        view([0, -90]);
    end
    % first anchor node is linearization pivot
    A = [pos_node(2:8,1) - pos_node(1,1), pos_node(2:8,2) - pos_node(1,2)];  
    d_sq = d_hat(2:8).^2;
    d_sq_ind = d_hat(2:8).^2;
    distance_between_an = pdist2(pos_node(2:8,:), pos_node(1,:)).^2;
    b = (d_hat(1)^2 - d_sq + distance_between_an)/2;
    b_ind = (d_hat_ind(1)^2 - d_sq_ind + distance_between_an)/2;
    x = (inv(A'*A)*A'*b)';
    x_ind = (inv(A'*A)*A'*b_ind)';
    plot(test_point(2), test_point(1), 'bd', x(1), x(2), 'r*'); hold off;
    view([0, -90]);
%     pause;
    test_point = [test_point(2), test_point(1)];
    e = test_point - x;
    e_ind = test_point - x_ind;
    e = sqrt(e(:,1).^2+ e(:,2).^2);
    e_ind = sqrt(e_ind(:,1).^2+ e_ind(:,2).^2);
    e_vec = [e_vec, e];
    e_vec_ind = [e_vec_ind, e_ind];
    end
end
mean(e_vec, 'omitnan')
mean(e_vec_ind, 'omitnan')
