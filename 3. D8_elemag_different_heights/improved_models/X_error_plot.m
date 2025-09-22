clear; clc
load('errors_at_6_11_17.mat')
X = -15:15;
figure
scatter(X,diff_H6_ori(:,1))
hold on
scatter(X,diff_H6_new(:,1))
xlabel('X location (cm)')
ylabel('X error (cm)')

figure
scatter(X,diff_H11_ori(:,1))
hold on
scatter(X,diff_H11_new(:,1))
xlabel('X location (cm)')
ylabel('X error (cm)')

figure
scatter(X,diff_H18_ori(:,1))
hold on
scatter(X,diff_H18_new(:,1))
xlabel('X location (cm)')
ylabel('X error (cm)')
