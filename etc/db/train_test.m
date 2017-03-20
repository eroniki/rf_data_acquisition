% Create a Fitting Network
hiddenLayerSize = [75, 50, 35, 25];
net = fitnet(hiddenLayerSize);
% net.layers{1}.transferFcn = 'logsig';
% net.layers{2}.transferFcn = 'tansig';
% net.layers{3}.transferFcn = 'elliotsig';
% net.layers{4}.transferFcn = 'satlins';
%%
% Set up Division of Data for Training, Validation, Testing
net.divideFcn = 'dividerand';
net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;

net.trainFcn = 'trainbr';
net.trainParam.epochs = 200000;
net.trainParam.showCommandLine = 1;
net.trainParam.max_fail = 0;
net.trainParam.goal = 1*10^-2; 
net.trainParam.show	= 1;

net.trainParam.mu_max = 1*10^40;
net.trainParam.min_grad = 1*10^-6;

net.performFcn='msereg';

%% % Train the Network
[net,tr] = train(net,data_lora',grid_labels_classifier','useParallel','yes','showResources','yes');

outputs = sim(net,data_lora');
outputs = outputs';

qq = outputs == grid_labels