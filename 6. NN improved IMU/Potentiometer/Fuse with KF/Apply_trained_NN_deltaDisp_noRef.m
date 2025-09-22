clear; clc
load('Trained_NN_oneMove.mat')
load('DataSet_forTest_2.mat')
% load('Moving_on_glass.mat')

%% Test for open loop (Have the real data as target) prediction for future
n_test = size(InputCell,1);
training_dim = 1:3;
% 1:n_test
for test_group_index = 1:n_test
XTest = (InputCell{test_group_index}(training_dim,1:end-1) - muX) ./ sigmaX;

net = resetState(net);
offset = 10;
[net,~] = predictAndUpdateState(net,XTest(:,1:offset));

numTimeSteps = size(XTest,2);
numPredictionTimeSteps = numTimeSteps - offset;
Y = 0;

% give the prediction by dynamic model as the input
for t = 1:numPredictionTimeSteps
    Xt = XTest(:,offset+t);   
    [net,Y(:,t)] = predictAndUpdateState(net,Xt);
end

D1 = cumsum(denormalize(Y,sigmaT,muT));
D3 = cumsum(InputCell{test_group_index}(3,offset:numTimeSteps-1));

figure
plot(cumsum(denormalize(XTest(3,1:offset),sigmaT,muT)),'--')
hold on
plot(offset:numTimeSteps-1, D1)
hold on
plot(offset:numTimeSteps-1, D3)
xlabel("Sampling")
ylabel('Disp (cm)')
legend("Input" , "Forecasted", "Raw")

% figure(2)
% error{test_group_index} =  sqrt(((D1-D2).^2));
% nexttile
% plot(error{test_group_index})
% hold on
% ylim([0 2])

end

%% Invert normalization to the actual scale
function Y = denormalize(X,sigma,mu)
    Y = X.*sigma + mu;
end