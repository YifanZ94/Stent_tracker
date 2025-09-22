clear; clc
load('Trained_NN_both_right_pad.mat')
% load('DataSet_test_fast_0.03.mat');

load('DataSet_test_slow.mat');

% load('DataSet_test_slow.mat')
% fast_data = load('DataSet_test_moveState.mat');
% InputCell = cat(1, InputCell, fast_data.InputCell);
% TargetCell = cat(1, TargetCell, fast_data.TargetCell);

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
training_dim = [1,2,3,4,6];   % new

% 1:n_test
for test_group_index = 9
XTest = (InputCell{test_group_index}(training_dim,1:end-1) - muX) ./ sigmaX;
TTest = (TargetCell{test_group_index}(:,2:end) - muT) ./ sigmaT;

net = resetState(net);
offset = 10;
[net,~] = predictAndUpdateState(net,XTest(:,1:offset));

numTimeSteps = size(XTest,2);
numPredictionTimeSteps = numTimeSteps - offset;
Y = 0;

state = InputCell{test_group_index}(4,1:end-1);
% give the prediction by dynamic model as the input
for t = 1:numPredictionTimeSteps
    Xt = XTest(:,offset+t);   
    [net,Y(:,t)] = predictAndUpdateState(net,Xt);
    
    if state(t) == 1
        Y(:,t) = 0;
    end
   
%     Y(:,t) = predict(net,Xt);
end

% D1 = denormalize(Y,sigmaT,muT);
% D2 = denormalize(TTest(offset:numTimeSteps-1),sigmaT,muT);
% D3 = XTest(3,offset:numTimeSteps-1)*sigmaX(3);

D1 = cumsum(denormalize(Y,sigmaT,muT));
D2 = cumsum(denormalize(TTest(offset:numTimeSteps-1),sigmaT,muT));
D3 = cumsum(XTest(3,offset:numTimeSteps-1)*sigmaX(3));

Ts = 0.015;
times = (1:size(D1,2))*Ts;
% offset:numTimeSteps-1

figure(1)
nexttile
plot(times, D1)
hold on
plot(times, D2,'--')
hold on
plot(times, D3)
hold off
xlabel("Time (s)")
ylabel('Displacement (m)')

% figure(2)
% error = abs(D1-D2);
% % scatter(abs(InputCell{test_group_index}(1,offset:numTimeSteps-1)), error)
% scatter(InputCell{test_group_index}(7,offset:numTimeSteps-1), error)  % uncorrected acc_amp
% hold on

k = find(state==1);
L_dynamic = size(k,2);
rmse_pred(test_group_index) = sum(sqrt((D1-D2).^2))/L_dynamic;
rmse_raw(test_group_index) = sum(sqrt((D2-D3).^2)/L_dynamic);

% rmse_pred(test_group_index) = sqrt(mean((D1-D2).^2));
% rmse_raw(test_group_index) = sqrt(mean((D2-D3).^2));

end

figure(1)
legend("Forecasted", "Ref", "Raw")

%% plot error
figure
plot(rmse_pred,'-o')
hold on
plot(rmse_raw,'-x')
xlabel('test group')
ylabel('RMSE')
legend('prediction','raw')
disp('done')

%% plot data
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