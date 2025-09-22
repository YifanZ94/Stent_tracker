clear; clc
load('Trained_NN_target_Error.mat')
load('DataSet_test_moveState.mat')

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
training_dim = [1,2,4,5,6];   % new aX, vX, deltSX, state, gyro_amp, acc_amp

% concatenate all tests
% n_test = 1;
% X = cat(2,InputCell{:});
% T = cat(2,TargetCell{:});

% 1:n_test
for test_group_index = 1:5
XTest = (InputCell{test_group_index}(training_dim,:) - muX) ./ sigmaX;
TTest = ((InputCell{test_group_index}(3,:) - TargetCell{test_group_index}) - muT) ./ sigmaT;

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
    
%     state = InputCell{test_group_index}(4,:);
%     if state(offset+t) == 0
%         Y(:,t) = predict(net,Xt);
%     else
%         Y(:,t) = 0;
%     end

end

E1 = abs(denormalize(Y,sigmaT,muT));
E2 = abs(denormalize(TTest(offset:numTimeSteps-1),sigmaT,muT));

Ts = 0.03;
times = (1:size(E1,2))*Ts;
% offset:numTimeSteps-1

% figure(1)
% nexttile
% plot(times, E1)
% hold on
% plot(times, E2,'--')
% hold off
% xlabel("Time (s)")
% ylabel('Error')

figure(2)
k = find(InputCell{test_group_index}(4,offset:numTimeSteps-1)==0);
% k = offset:numTimeSteps-1;
scatter(abs(InputCell{test_group_index}(7,offset:numTimeSteps-1)), E2)
% scatter(InputCell{test_group_index}(7,k), E1(k))  % uncorrected acc_amp
xlabel('dynamic |a| (m/s^2)')
ylabel('Error (m)')


hold on

% nexttile
% plot(error{test_group_index})
% hold on
% ylim([0 2])

% RMSE in Error
rmse_pred(test_group_index) = sqrt(mean((E1 - E2).^2));
end
% figure(1)
% legend("Predicted", "Ref")

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
% figure
% subplot(411)
% plot(times,XTest(1,offset+1:end))
% ylabel('a_x (m/s^2)')
% subplot(412)
% plot(times,XTest(2,offset+1:end))
% ylabel('v_x (m/s)')
% subplot(413)
% plot(times,XTest(3,offset+1:end))
% ylabel('\Delta s_x (cm)')
% subplot(414)
% plot(times,InputCell{test_group_index}(5,offset+2:end))
% xlabel('Time (s)')
% ylabel('|\omega| (\circ/s)')

%% Invert normalization to the actual scale
function Y = denormalize(X,sigma,mu)
    Y = X*sigma + mu;
end