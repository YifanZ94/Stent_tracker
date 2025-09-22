clear; clc
% load('B_10-30.mat');   % normalized
load('B2d_5layers.mat');

data = load('H18_ideal_mag.txt');

% data3 = load('MMC_21.7_env.txt');
% env3 = load('MMC_21.7_mag.txt');
% data = data3 - env3;
%% model / real ratio
magnitude_map = sqrt(Brho.^2 + Bz.^2);
magnitude_test = sqrt(data(:,1).^2 + data(:,2).^2 + data(:,3).^2);
H_index = 17;  %% (17 for H=18, 31 for H=25)

C1 = magnitude_test(H_index,:)./magnitude_map(H_index,:);
C2 = data(:,3)'./Bz(H_index,:);   % when dim = 1, change Bz to Brho
C3 = data(:,1)'./Brho(H_index,:);
C3(31) = NaN;
x = -15:0.5:15;
%%
plot(x,C1)    %% searched with C=2.1
xlabel('location(cm)')
title('magnitude ratio')
figure
plot(x,C2)
title('Z reading ratio')
xlabel('location(cm)')
figure
plot(x,C3)
title('X reading ratio')
xlabel('location(cm)')
figure
plot(x,data(:,2))
title('Y net reading(Gauss)')
xlabel('location(cm)')
%% relation with distance
relative_dis = sqrt(x.^2 + 18^2);
figure
C_ref = C1(31);
plot(x,(relative_dis/18).^3, x,C1/C_ref)
legend('distance','mag magnitude')
xlabel('location(cm)')
%%  convert real to model
% C = 2.1;
% dim = 3;
% figure
% plot(x,data(:,dim),x,C*Bz(17,:))
% xlabel('location(cm)')
% legend('real test','magnet model')
% title('height at 18')