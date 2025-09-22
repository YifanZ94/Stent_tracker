clear; clc
load('DataSet_train_fast_0.03.mat')
test_data = load('DataSet_test_fast_0.03.mat');

train_inputs_set = cat(1, InputCell, test_data.InputCell);
train_target_set = cat(1, TargetCell, test_data.TargetCell);
idxTrain = size(train_inputs_set,1);

%%
for i = 1:idxTrain
    accX = train_inputs_set{i}(1,:);
    dyn_state = train_inputs_set{i}(4,:);
    k = find(dyn_state==1);
    dyn_accX = accX;
    dyn_accX(k) = [];
    dyn_accX_mean(i) = mean(abs(dyn_accX));
    dyn_accX_max(i) = max(abs(dyn_accX));
end

%% slow
load('DataSet_train_slow_0.015.mat')
test_data = load('DataSet_test_slow.mat');
train_inputs_set = cat(1, InputCell, test_data.InputCell);
train_target_set = cat(1, TargetCell, test_data.TargetCell);
idxTrain2 = size(train_inputs_set,1);

%%
for i = 1:idxTrain2
    accX = train_inputs_set{i}(1,:);
    dyn_state = train_inputs_set{i}(4,:);
    k = find(dyn_state==1);
    dyn_accX = accX;
    dyn_accX(k) = [];
    dyn_accX_mean_2(i) = mean(abs(dyn_accX));
    dyn_accX_max_2(i) = max(abs(dyn_accX));
end

%%
figure
scatter(1:idxTrain, dyn_accX_mean,'bx')
hold on
scatter(1:idxTrain2, dyn_accX_mean_2, 'bx')
xlabel('test group')
ylabel('|a_x| mean (m/s^2)')
legend('fast group', 'slow group')

figure
scatter(1:idxTrain, dyn_accX_max)
hold on
scatter(1:idxTrain2, dyn_accX_max_2, 'x')
xlabel('test group')
ylabel('|a_x| max (m/s^2)')
legend('fast group', 'slow group')