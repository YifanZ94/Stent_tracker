clear; clc
load('Trained_NN_concat_5.mat')
% load('test_fast2.mat')
load('test_slow.mat')

% muX = [0;0;0];
% muT = 0;
%% Test for open loop (Have the real data as target) prediction for future
n_test = size(InputCell,1);
training_dim = [1,2,3,4,5];
% 1:n_test
for test_group_index = 1:n_test
XTest = (InputCell{test_group_index}(training_dim,1:end-1) - muX) ./ sigmaX;
% XTest = InputCell{test_group_index}(training_dim,1:end-1);

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
% D1 = cumsum(Y);
D3 = cumsum(InputCell{test_group_index}(3,offset:numTimeSteps-1));

times = InputCell{test_group_index}(4,offset:end-2);
% offset:numTimeSteps-1

figure(1)
nexttile
% plot(cumsum(denormalize(TTest(1,1:offset),sigmaT,muT)))
% hold on

plot(times, D1)
hold on
plot(times, D3)
hold off
xlabel("Time (s)")
ylabel('Displacement (cm)')
legend("Forecasted",  "Raw")

% figure(2)
% error{test_group_index} =  sqrt(((D1-D2).^2));
% nexttile
% plot(error{test_group_index})
% hold on
% ylim([0 2])

% rmse_pred(test_group_index) = sqrt(mean((D1-D2).^2))/size(D1,2);
% rmse_raw(test_group_index) = sqrt(mean((D2-D3).^2))/size(D1,2);

end

%%
% figure
% plot(rmse_pred,'-o')
% hold on
% plot(rmse_raw,'-x')
% xlabel('test group')
% ylabel('RMSE')
% legend('prediction','raw')
% disp('done')

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