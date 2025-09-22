clear; clc
data1 = load('DataSet_train_fast_0.03.mat');
data2 = load('DataSet_train_slow_0.015.mat');
data3 = load('DataSet_test_fast_0.03.mat');
data4 = load('DataSet_test_slow_0.015.mat');
Input = cat(1, data1.InputCell, data2.InputCell, data3.InputCell, data4.InputCell);
Target = cat(1, data1.TargetCell, data2.TargetCell, data3.TargetCell, data4.TargetCell);

%% split
% for i = 1:size(Input,1)
%     inputs = cell2mat(Input(i));
%     targets = cell2mat(Target(i));
%     save(sprintf('Input%d',i), 'inputs');
%     save(sprintf('Target%d',i), 'targets');
% end

%% concat
raw_seq = cell2mat(Input(1));
new_seq = cell2mat(Target(1));

for i = 2:size(Input,1)
    mat = cell2mat(Input(i));
    raw_seq = [raw_seq, mat(1:6,:)];
    new_seq = [new_seq, cell2mat(Target(i))];
end




