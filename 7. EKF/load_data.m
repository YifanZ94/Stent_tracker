clear; clc
data1 = load('Test1_KF_ori.mat');
data2 = load('Test1_KF_0SawDistb.mat');

X1 = data1.X_post;
X2 = data2.X_post;

figure
plot(X1)
hold on
plot(X2)