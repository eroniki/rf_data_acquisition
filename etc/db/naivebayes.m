clc; close all;
 
precision = @(confusionMat) diag(confusionMat)./sum(confusionMat,2);
recall = @(confusionMat) diag(confusionMat)./sum(confusionMat,1)';
f1Scores = @(confusionMat) 2*(precision(confusionMat).*recall(confusionMat))./(precision(confusionMat)+recall(confusionMat));
meanF1 = @(confusionMat) mean(f1Scores(confusionMat));

X = data_lora;
% Y = categorical(grid_labels_linear);[grid_labels(:,1),ceil(grid_labels(:,1)/2)]
Y_x = grid_labels(:,1);
Y_y = grid_labels(:,2);
% Y_x = ceil(grid_labels(:,1)/2);
% Y_y = ceil(grid_labels(:,2)/2);
grid_labels_ds= [Y_x, Y_y];
Mdl_x = fitcnb(X,Y_x, 'DistributionNames', 'mvmn');
Mdl_y = fitcnb(X,Y_y, 'DistributionNames', 'mvmn');

[y_hat_ind_x, post_x, ~] = Mdl_x.predict(X);
lbls_x = Mdl_x.ClassNames;

y_hat_x = lbls_x(y_hat_ind_x);

[y_hat_ind_y, post_y, ~] = Mdl_y.predict(X);
lbls_y = Mdl_y.ClassNames;

y_hat_y = lbls_y(y_hat_ind_y);


[valx, idx] = max(post_x, [], 2);
[valy, idy] = max(post_y, [], 2);

[grid_labels, idx, idy];
% [xx, yy] = meshgrid(1:7, 1:25);
% for i=1:1543
%     px = post_x(i,:)';
%     py = post_y(i,:);
%     ppx = repmat(px, [1, 25]);
%     ppy = repmat(py, [7, 1]);
%     post = (px*py);
%     post = post/sum(post(:));
%     [val, id] = max(post(:));
%     [u, v] = ind2sub(size(post), id);
%     disp([grid_labels(i,:), u, v]);
%     figure(1);
%     subplot(1,2,1); imagesc(ppx');
%     subplot(1,2,2); imagesc(ppy');
%     figure(2);
%     poster = ppx.*ppy;
%     poster = poster / sum(poster(:));
%     view([0, -90]);
%     surf(xx,yy,poster');
%     figure(1);
%     subplot(2,1,1); bar(px);
%     subplot(2,1,2); bar(py);
%     figure(2);
%     imagesc(post);
%     view([0, -90]);
%     pause;
% end
[confusionMatx,order] = confusionmat(Y_x,y_hat_x);
[confusionMaty,order] = confusionmat(Y_y,y_hat_y);

% precision(confusionMatx)
recall(confusionMatx);
% f1Scores(confusionMatx)
% meanF1(confusionMatx)

% precision(confusionMaty)
recall(confusionMaty);
% f1Scores(confusionMaty)
% meanF1(confusionMaty)

subplot(1,2,1); imagesc(confusionMatx/1543);
subplot(1,2,2); imagesc(confusionMaty/1543);

sum(bsxfun(@eq, Y_x,double(y_hat_x)))/1543
sum(bsxfun(@eq, Y_y,double(y_hat_y)))/1543
I = sum(grid_labels_ds(:, 1) == y_hat_x & grid_labels_ds(:, 2) == y_hat_y)/1543

e_bayesian = localization_error(grid_labels, [y_hat_x, y_hat_y]);

[cdf, counts, bins] = localization_cdf(e_vec(:), 100);
[cdf_ind, counts_ind, bins_ind] = localization_cdf(e_vec_ind(:), 100);
[cdf_b, counts_b, bins_b] = localization_cdf(e_bayesian(:), 100);
figure; plot(bins, cdf, 'r', bins_ind, cdf_ind, 'b', bins_b, cdf_b, 'g'); grid on; grid minor; 
legend('Joint PLE', 'Ind PLE', 'Bayesian');

[min_val, min_id] = min(abs(cdf-0.9));
bins(min_id)

[min_val, min_id] = min(abs(cdf_ind-0.9));
bins_ind(min_id)

[min_val, min_id] = min(abs(cdf_b-0.9));
bins_b(min_id)