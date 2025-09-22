clear; clc
load('Square_X40_Y20_Z15_reso0.5.mat');
B_old_amp = sqrt(BX.^2+BY.^2+BZ.^2);
load('Square_X40_Y20_Z15_reso0.5_T.mat');
B_new_amp = sqrt(BX.^2+BY.^2+BZ.^2);

Y_ind = 21;
B_ratio = B_new_amp./B_old_amp;
B_ratio_Y = squeeze(B_ratio(:,Y_ind,:));

[X,Y] = meshgrid(-20:0.5:20, -5:-0.5:-20);
surf(X,Y,B_ratio_Y')
colorbar
xlabel('X (cm)')
ylabel('Z (cm)')
