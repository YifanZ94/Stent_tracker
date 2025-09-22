clear; clc
load('DataSet_concat.mat')

%% normalize by mean and var
X = raw_seq;
T = new_seq;
training_dim = 6;
L = size(X,2);

muX = zeros(size(training_dim,2),1);
muT = 0;

% sigmaX = std(X,0,2);
% sigmaT = std(T,0,2);

sigmaX = max(X,[],2) - min(X,[],2);
sigmaT = max(T) - min(T);

X = X./sigmaX;
T = T./sigmaT;

%%
windowSize = 30;
nrows = L - windowSize;
features = cell(nrows, 1);

for row = 1:nrows
    features{row} = X(:,row:windowSize+row-1);
end

response = T(:,windowSize+1:end)';
nTest = round(0.7*L);

xTest = features(end-nTest+1:end,:);
yTest = response(end-nTest+1:end,:);

xTrain = features(1:end-nTest,:);
yTrain = response(1:end-nTest,:);

%%  Define the NN architecture
numChannels = size(raw_seq, 1);
maxPosition = 128;
numHeads = 4;
numKeyChannels = numHeads*32;

layers = [
    sequenceInputLayer(numChannels, Name="input")

    positionEmbeddingLayer(numChannels, maxPosition, Name="pos-emb");
    additionLayer(2, Name="add")

    selfAttentionLayer(numHeads,numKeyChannels,'AttentionMask','causal')
    % selfAttentionLayer(numHeads,numKeyChannels)

    indexing1dLayer("last")
    gruLayer(70,'OutputMode','sequence')

    fullyConnectedLayer(40)
    dropoutLayer(0.6)
    fullyConnectedLayer(1)

    regressionLayer];

lgraph = layerGraph(layers);
lgraph = connectLayers(lgraph, "input", "add/in2");

options = trainingOptions("adam", ...
    MaxEpochs= 500, ...
    miniBatchSize = 32, ...
    SequencePaddingDirection="right", ...
    InitialLearnRate= 0.01, ...
    GradientThreshold= 0.5, ...
    Shuffle="every-epoch", ...
    Plots="training-progress", ...
    Verbose=0);

%% Train
[net,info] = trainNetwork(xTrain,yTrain,lgraph,options);
% YTest = predict(net,XTest,SequencePaddingDirection="left");
%%
T_RMSE = info.TrainingRMSE;
X = 1:size(T_RMSE,2);
semilogx(X,T_RMSE)
xlim([0 2000])
xlabel('Epochs')
ylabel('RMSE')

%% Test for open loop (Have the real data as target) prediction for future
data = load('DataSet_test_fast_0.03.mat');
data2 = load('DataSet_test_slow_0.015.mat');
InputCell = cat(1, data.InputCell, data2.InputCell);
TargetCell = cat(1, data.TargetCell, data2.TargetCell);
test_inputs_set = InputCell;
test_target_set = TargetCell;
n_test = size(test_inputs_set,1);
targer_dim = 1;

% 1:n_test
for test_group_index = 1:2:n_test
cat_test_input = test_inputs_set{test_group_index};
cat_test_target = test_target_set{test_group_index};

XTest = cat_test_input(training_dim,1:end-1)./ sigmaX;
TTest = cat_test_target(targer_dim,2:end) ./ sigmaT;

net = resetState(net);
offset = 10;
[net,~] = predictAndUpdateState(net,XTest(:,1:offset));

numTimeSteps = size(XTest,2);
numPredictionTimeSteps = numTimeSteps - offset;
Y = 0;

% give the prediction by dynamic model as the input
for t = 1:numPredictionTimeSteps
    Xt = XTest(:,offset+t);   
    [net,Y(1,t)] = predictAndUpdateState(net,Xt);
    %     Y(:,t) = predict(net,Xt);
end

% D1 = denormalize(Y,sigmaT,muT);
% D2 = denormalize(TTest(offset:numTimeSteps-1),sigmaT,muT);
% D3 = test_inputs_set{test_group_index}(3,offset:numTimeSteps-1);

D1 = cumsum(denormalize(Y,sigmaT,muT));
D2 = cumsum(denormalize(TTest(offset:numTimeSteps-1),sigmaT,muT));
D3 = cumsum(test_inputs_set{test_group_index}(3,offset:numTimeSteps-1));

figure(1)
nexttile
plot(offset:numTimeSteps-1, D1)
hold on
plot(offset:numTimeSteps-1, D2, '--')
hold on
plot(offset:numTimeSteps-1, D3)
hold off
xlabel("Sampling")
ylabel('displacement (cm)')

% rmse_pred(test_group_index) = sqrt(mean((D1-D2).^2));
% rmse_raw(test_group_index) = sqrt(mean((D2-D3).^2));

dynm_idx = max(1, find(cat_test_input(4,:)==0) - 11);
rmse_pred(test_group_index) = sqrt(mean((D1(dynm_idx)-D2(dynm_idx)).^2));
rmse_raw(test_group_index) = sqrt(mean((D2(dynm_idx)-D3(dynm_idx)).^2));

end

legend("Forecasted", "Ref", 'Raw')

%%
figure
plot(rmse_pred,'-o')
hold on
plot(rmse_raw,'-x')
xlabel('test group')
ylabel('RMSE (m)')
legend('prediction','raw')
disp('done')

%%
disp('done')
save('Trained_NN_both_standard.mat', 'net', 'muT', 'muX','sigmaT','sigmaX')

%% compare with NN in py
% rmse_py = load('GRU1_py_rmse.txt');
% % rmse_pred = load('GRU2_rmse.txt');
% figure
% plot(rmse_pred)
% hold on
% plot(rmse_py*sqrt(sigmaT))
% % plot(rmse_py)
% xlabel('test group')
% ylabel('RMSE (m)')
% legend('MATLAB', 'Py')

%% Invert normalization to the actual scale
function Y = denormalize(X,sigma,mu)
    Y = X*sigma + mu;
end
