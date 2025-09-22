clear; clc
load('Trained_NN.mat')
load('DataSet_test_slow.mat')

T_thres = 4.5e-4;
n_test = size(InputCell,1);
for n = 1:n_test
    T = TargetCell{n};
    T_state = 0;
    for j = 1:size(T,2)
        if abs(T(j)) < T_thres
            T_state(j) = 0;
        else
            T_state(j) = 1;
        end
    end
    TargetCell{n,2} = categorical(T_state);
end

%% Test for open loop (Have the real data as target) prediction for future
training_dim = [1,2,3,6,7];
miniBatchSize = 10;

% n_test
for test_group_index = 1:n_test
XTest = InputCell{test_group_index}(training_dim,:) ;
TTest = TargetCell{test_group_index,2} ;

YPred{test_group_index} = classify(net,XTest,'MiniBatchSize',miniBatchSize);
acc_pred(test_group_index) = mean(YPred{test_group_index} == TTest);
acc_dynamics(test_group_index) = mean(categorical(InputCell{test_group_index}(4,:)) ~= TTest);

% clear YPred2_temp
% current_state = 0;
% start_ind = 1;
% for sample = 1:size(TTest,2)
%     YPred2_temp(:,sample) = classify(net,XTest(:,sample),'MiniBatchSize',miniBatchSize);
% end
% YPred2{test_group_index} = YPred2_temp;
% acc_pred2(test_group_index) = mean(YPred2{test_group_index} == TTest);

end

disp('done')

%%
figure
n = 2;
subplot(411)
plot(TargetCell{n,1})
hold on
plot([1 size(TargetCell{n,1},2)], [T_thres  T_thres])
subplot(412)
plot(TargetCell{n,2})
subplot(413)
plot(YPred{n})
ylabel('pred')
subplot(414)
plot(categorical(~InputCell{n}(4,:)))
ylabel('dynamic')
xlabel('sample')

%%
figure
plot(acc_pred, '-o')
hold on
plot(acc_dynamics, '-o')
% hold on
% plot(acc_pred2, '-o')
legend('prediction', 'dynamics')
ylabel('Accuracy')
xlabel('Test group')

%%
% figure
% for i = 1:10
%     nexttile
%     plot(YPred{i},'o')
%     hold on
%     plot(YPred2{i})
% end
% legend('all', 'iterative')
%% Invert normalization to the actual scale
function Y = denormalize(X,sigma,mu)
    Y = X*sigma + mu;
end