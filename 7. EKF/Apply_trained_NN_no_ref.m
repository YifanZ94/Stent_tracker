clear; clc
load('Trained_NN_both_standard.mat')
load('test_3.mat');
% load('test_slow.mat')

%%
denormalize = @(X,sigma,mu) X*sigma + mu;

%% Test for open loop (Have the real data as target) prediction for future
n_test = size(InputCell,1);
% training_dim = [1,2,3,4];   % for old set
training_dim = [1,2,3,4,6];   % new

sigmaX(4) = 1;
% 1:n_test
for test_group_index = 1:n_test
XTest = (InputCell{test_group_index}(training_dim,1:end-1) - muX) ./ (sigmaX);

Ts = InputCell{1}(5,2)-InputCell{1}(5,1);

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
D3 = cumsum(XTest(3,offset:numTimeSteps-1)*sigmaX(3));

% offset:numTimeSteps-1
times = (1:size(D1,2))*Ts;

figure(1)
nexttile
plot(times, D1)
hold on
plot(times, D3)
hold off
xlabel("Time (s)")
ylabel('Displacement (m)')

end

legend("Forecasted", "Raw")


%% plot data
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
