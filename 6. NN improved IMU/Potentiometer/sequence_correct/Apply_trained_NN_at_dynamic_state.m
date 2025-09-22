clear; clc
load('DataSet_test_slow.mat')
load('Trained_NN_Gru_slow.mat')

% test_set2 = load('DataSet_test_ori.mat').InputCell;
% group_ind = 1;
% plot(100*InputCell{group_ind}(1,:))
% hold on
% plot(test_set2{group_ind}(1,:))
% muX = [0;0;0];
% muT = 0;
%% Test for open loop (Have the real data as target) prediction for future
n_test = size(InputCell,1);
% training_dim = [1,2,3,4];   % for old set
training_dim = [1,2,3,4,5];   % new

% concatenate all tests
% n_test = 1;
% X = cat(2,InputCell{:});
% T = cat(2,TargetCell{:});

% 1:n_test
for test_group_index = 1:n_test
XTest = (InputCell{test_group_index}(training_dim,1:end-1) - muX) ./ sigmaX;
TTest = (TargetCell{test_group_index}(:,1:end-1) - muT) ./ sigmaT;

net = resetState(net);
offset = 10;
[net,~] = predictAndUpdateState(net,XTest(:,1:offset));

numTimeSteps = size(XTest,2);
numPredictionTimeSteps = numTimeSteps - offset;
Y = 0;

% give the prediction by dynamic model as the input
for t = 1:numPredictionTimeSteps
    Xt = XTest(:,offset+t);   
%     [net,Y(:,t)] = predictAndUpdateState(net,Xt);
    Y(:,t) = predict(net,Xt);
end

delta_D1 = denormalize(Y,sigmaT,muT);
delta_D2 = denormalize(TTest(offset:numTimeSteps-1),sigmaT,muT);
delta_D3 = InputCell{test_group_index}(3,offset:numTimeSteps-1);

D1 = cumsum(delta_D1);
D2 = cumsum(delta_D2);
D3 = cumsum(delta_D3);

Ts = 0.03;
times = (1:size(D1,2))*Ts;
% offset:numTimeSteps-1

% figure(1)
% nexttile
% plot(cumsum(denormalize(TTest(1,1:offset),sigmaT,muT)))
% hold on

% plot(times, delta_D1)
% hold on
% plot(times, delta_D2,'--')
% hold on
% plot(times, delta_D3)
% hold off
% xlabel("Time (s)")
% ylabel('Displacement (cm)')

figure(2)
error = sqrt(((delta_D1-delta_D2).^2));
% scatter(abs(InputCell{test_group_index}(1,offset:numTimeSteps-1)), error)
scatter(InputCell{test_group_index}(7,offset:numTimeSteps-1), error)  % uncorrected acc_amp
hold on

% nexttile
% plot(error{test_group_index})
% hold on
% ylim([0 2])

% RMSE in overall disp
% rmse_pred(test_group_index) = sqrt(mean((D1-D2).^2));
% rmse_raw(test_group_index) = sqrt(mean((D2-D3).^2));

% RMSE in Delta disp
% rmse_pred(test_group_index) = sqrt(mean((delta_D1 - delta_D2).^2));
% rmse_raw(test_group_index) = sqrt(mean((delta_D2 - delta_D3).^2));

end

figure(1)
legend("Forecasted", "Ref", "Raw")

%%
figure
plot(rmse_pred,'-o')
hold on
plot(rmse_raw,'-x')
xlabel('test group')
ylabel('RMSE')
legend('prediction','raw')
disp('done')

%%
figure
subplot(411)
plot(times,XTest(1,offset+1:end))
ylabel('a_x (m/s^2)')
subplot(412)
plot(times,XTest(2,offset+1:end))
ylabel('v_x (m/s)')
subplot(413)
plot(times,XTest(3,offset+1:end))
ylabel('\Delta s_x (cm)')
subplot(414)
plot(times,InputCell{test_group_index}(5,offset+2:end))
xlabel('Time (s)')
ylabel('|\omega| (\circ/s)')

%% Invert normalization to the actual scale
function Y = denormalize(X,sigma,mu)
    Y = X*sigma + mu;
end