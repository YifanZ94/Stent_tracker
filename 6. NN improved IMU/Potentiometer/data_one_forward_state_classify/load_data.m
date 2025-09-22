clear; clc
load('DataSet1.mat');
inputs_1 = InputCell;
target_1 = TargetCell;
load('DataSet2_delta.mat');
inputs_2 = InputCell;
target_2 = TargetCell;
%%
figure
dim = 3;
for i = 1:10
    nexttile
    disp_IMU_1 = inputs_1{i}(dim,:);
    disp_IMU_2 = cumsum(inputs_2{i}(dim,:));
    plot(disp_IMU_1,'o')
    hold on
    plot(disp_IMU_2,'x')
end

figure
for i = 1:10
    nexttile
    disp_ref_1 = target_1{i};
    disp_ref_2 = cumsum(target_2{i});
    plot(disp_ref_1,'o')
    hold on
    plot(disp_ref_2,'x')
end
