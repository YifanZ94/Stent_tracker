clear; clc
load('DataSet.mat')
%%
% inputs: X(acc,v,disp), gyro, Ts
% target: displacement, velocity, acc ref
training_dim = 1:3;
% training_dim = [1:3,7];
targer_dim = 1;
idxTrain = floor(0.75*size(InputCell,1));
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

X = cat(2,XTrain{:});
T = cat(2,TTrain{:});
figure
plot(X')
figure
plot(T)

%% normalize by mean and var

minX = min(X,[],2);
maxX = max(X,[],2);
minT = min(T,[],2);
maxT = max(T,[],2);

minX = [-155;150;-40];
maxX = [160;120;90];

for n = 1:numel(XTrain)
    XTrain{n,1} = (XTrain{n} - minX) ./ (maxX - minX);
    TTrain{n,1} = (TTrain{n} - minT) ./ (maxT - minT);
end
 
%%  Define the NN architecture
layers = [
    sequenceInputLayer(numel(training_dim))
    lstmLayer(100,'OutputMode','sequence')
    fullyConnectedLayer(50)
    dropoutLayer(0.6)
    fullyConnectedLayer(numel(targer_dim))
    regressionLayer];

options = trainingOptions("adam", ...
    MaxEpochs= 70, ...
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
% :n_test
for test_group_index = 1:n_test
cat_test_input = test_inputs_set{test_group_index};
cat_test_target = test_target_set{test_group_index};

XTest = (cat_test_input(training_dim,1:end-1) - minX) ./ (maxX - minX);
TTest = (cat_test_target(1,2:end) - minT) ./ (maxT - minT);

net = resetState(net);
offset = 5;
[net,~] = predictAndUpdateState(net,XTest);

numTimeSteps = size(XTest,2);
numPredictionTimeSteps = numTimeSteps - offset;
Y = 0;

% give the prediction by dynamic model as the input
for t = 1:numPredictionTimeSteps
    Xt = XTest(:,offset+t);   
    [net,Y(:,t)] = predictAndUpdateState(net,Xt);
end

D1 = denormalize(Y,maxX(3),minX(3));
D2 = denormalize(TTest(offset:numTimeSteps-1),maxT,minT);
D3 = test_inputs_set{test_group_index}(3,offset:numTimeSteps-1);
% 

figure
plot(denormalize(TTest(1,1:offset),maxX(3),minX(3)))
hold on
plot(offset:numTimeSteps-1, D1)
hold on
plot(offset:numTimeSteps-1, D2,'--')
hold on
plot(offset:numTimeSteps-1, D3)
hold off
xlabel("Sampling")
ylabel('Velocity (m/s2)')
legend("Input" , "Forecasted", "Ref", "Raw")

rmse_pred(test_group_index) = sqrt(mean((D1-D2).^2));
rmse_raw(test_group_index) = sqrt(mean((D2-D3).^2));
end
disp('done')

%% RMSE comparison
[B,I] = sort(acc_amp);
figure
plot(rmse_raw(I),'o--')
hold on
plot(rmse_pred(I),'x--')
legend('raw','prediction')
ylabel('RMSE')
xlabel('Test number')

%% Invert normalization to the actual scale

function Y = denormalize(X, maxX, minX)
    Y = X.*(maxX-minX) + minX;
end