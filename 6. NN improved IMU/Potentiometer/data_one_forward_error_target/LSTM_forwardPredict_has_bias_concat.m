clear; clc
load('DataSet_train_ori.mat')
%%
% inputs: X(acc,v,disp), gyro, Ts
% target: displacement, velocity, acc ref
training_dim = 1:4;
% training_dim = [1:3,7];
targer_dim = 1;
idxTrain = floor(0.9*size(InputCell,1));
train_inputs_set = InputCell(1:idxTrain);
train_target_set = TargetCell(1:idxTrain);
test_inputs_set = InputCell(idxTrain+1:end);
test_target_set = TargetCell(idxTrain+1:end);

%% prepare data for traning
n_training = size(train_inputs_set,1);
n_test = size(test_inputs_set,1);

for n = 1:n_training
    X = train_inputs_set{n};
    XTrain{n,1} = X(training_dim,1:end-1);
    T = train_target_set{n};
    TTrain{n,1} = T(targer_dim,2:end);
end

%% normalize by mean and var
X = cat(2,XTrain{:});
T = cat(2,TTrain{:});
% muX = mean(X,2);
% muT = mean(T,2);

muX = [0;0;0;0];
muT = 0;
sigmaX = std(X,0,2);
sigmaT = std(T,0,2);

cat_L = 5;

for n = 1:numel(XTrain) - cat_L
%     XTrain{n,1} = (XTrain{n} - muX) ./ sigmaX;
%     TTrain{n,1} = (TTrain{n} - muT) ./ sigmaT;
    XTrain{n,1} = cat(2,XTrain{n:n+cat_L-1}) ./ sigmaX;
    TTrain{n,1} = cat(2,TTrain{n:n+cat_L-1}) ./ sigmaT;
end
 
%%  Define the NN architecture
layers = [
    sequenceInputLayer(numel(training_dim))
%     lstmLayer(100,'OutputMode','sequence')
    gruLayer(50,'OutputMode','sequence')

%     imageInputLayer([7 28 1])
%     convolution2dLayer([3 5], 4) 
%     reluLayer
%     maxPooling2dLayer(3,'Stride',3)
          
    fullyConnectedLayer(60)
    dropoutLayer(0.6)
    fullyConnectedLayer(numel(targer_dim))
    regressionLayer];

options = trainingOptions("adam", ...
    MaxEpochs= 200, ...
    SequencePaddingDirection="left", ...
    InitialLearnRate= 0.02, ...
    GradientThreshold=1, ...
    Shuffle="every-epoch", ...
    Plots="training-progress", ...
    Verbose=0);

%% Train
net = trainNetwork(XTrain,TTrain,layers,options);
% YTest = predict(net,XTest,SequencePaddingDirection="left");

%% Test for open loop (Have the real data as target) prediction for future
% 1:n_test
for test_group_index = 1:n_test
cat_test_input = test_inputs_set{test_group_index};
cat_test_target = test_target_set{test_group_index};

XTest = (cat_test_input(training_dim,1:end-1) - muX) ./ sigmaX;
TTest = (cat_test_target(targer_dim,2:end) - muT) ./ sigmaT;

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
end

D1 = denormalize(Y,sigmaT,muT);
prediction{test_group_index} = D1;
D2 = denormalize(TTest(offset:numTimeSteps-1),sigmaT,muT);
D3 = test_inputs_set{test_group_index}(3,offset:numTimeSteps-1);

figure
plot((denormalize(TTest(1,1:offset),sigmaT,muT)))
hold on
plot(offset:numTimeSteps-1, (D1))
hold on
plot(offset:numTimeSteps-1, (D2), '--')
hold on
plot(offset:numTimeSteps-1, D3)
hold off
xlabel("Sampling")
ylabel('displacement (cm)')
legend("Input" , "Forecasted", "Ref", 'Raw')

rmse_pred(test_group_index) = sqrt(mean((D1-D2).^2));
rmse_raw(test_group_index) = sqrt(mean((D2-D3).^2));
end

disp('done')
save('Trained_NN.mat', 'net', 'muT','muX','sigmaT','sigmaX')

%% Invert normalization to the actual scale
function Y = denormalize(X,sigma,mu)
    Y = X*sigma + mu;
end

% function Y = denormalize(X, maxX, minX)
%     Y = X.*(maxX-minX) + minX;
% end