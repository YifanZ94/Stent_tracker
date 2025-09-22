clear; clc
load('Square_X40_Y20_Z15_reso0.5.mat');
% load('Square_X40_Y20_Z15_reso0.5_T.mat');
file = {'H6.5_LIS.txt';
    'H8_LIS.txt';
    'H10_LIS.txt';
    'H11_LIS.txt';
    'H13_LIS.txt';
    'H15_LIS.txt';
    'H17_LIS.txt'};

%% model / real ratio
magnitude_map = sqrt(BX.^2 + BY.^2 + BZ.^2);
Z_ind_list = {4,7,11,13,17,21,25};
Y_ind_list = {-1,-1,-1,1,0,0,-1};
X_ind_list = {1,-1,0,1,-1,0,0};
ratio_var = zeros(7,1);

for test_group = [1,5,7]
    data = load(file{test_group});
    magnitude_test = sqrt(data(:,1).^2 + data(:,2).^2 + data(:,3).^2);
Z_index = Z_ind_list{test_group};    % 7, 13, 21
Y_index = 21 + Y_ind_list{test_group};
x = linspace(-15,15,size(data,1));
X_index = linspace(11,71,size(data,1)) + X_ind_list{test_group};

C_magnitude = magnitude_test./magnitude_map(X_index,Y_index,Z_index);
C_z = data(:,3)./BZ(X_index,Y_index,Z_index);   % when dim = 1, change Bz to Brho
C_x = data(:,1)./BX(X_index,Y_index,Z_index);
C_y = data(:,2)./BY(X_index,Y_index,Z_index);
C_x(round(size(C_x,1)/2)) = 0;

ratio_var(test_group,1) = var(C_magnitude/1.7, 0, 1);

plot(x,C_magnitude,'-x')
hold on
xlabel('X position(cm)')
ylabel('test/model amplitude ratio')
end

legend('H=6.5','H=11','H=18')
set(gca,'FontSize',13)
%% plot mag data
% figure
% C = median(C_z);
% C2 = median(C_x);
% subplot(131)
% plot(x,data(:,1),x,C*BX(X_index,Y_index,Z_index))
% legend('test','ref')
% title('test and ref')
% subplot(132)
% plot(x,data(:,2),x,C*BY(X_index,Y_index,Z_index))
% subplot(133)
% plot(x,data(:,3),x,C*BZ(X_index,Y_index,Z_index))

%% abs difference
% figure
% subplot(131)
% plot(x,data(:,1) - C*BX(X_index,Y_index,Z_index))
% title('difference in X')
% subplot(132)
% plot(x,data(:,2)- C*BY(X_index,Y_index,Z_index))
% title('difference in Y')
% subplot(133)
% plot(x,data(:,3)- C*BZ(X_index,Y_index,Z_index))
% title('difference in Z')

%% plot the ratio
% figure
% subplot(221)
% scatter(x,1./C_magnitude)
% xlabel('location(cm)')
% title('magnitude ratio')
% subplot(222)
% scatter(x,1./C_z)
% title('Z reading ratio')
% xlabel('location(cm)')
% subplot(223)
% scatter(x,1./C_x)
% title('X reading ratio')
% xlabel('location(cm)')
% subplot(224)
% scatter(x,1./C_y)
% title('Y reading ratio')
% xlabel('location(cm)')

%%
% C = 2.1;
% figure
% scatter(x,C_magnitude/C)
% xlabel('location(cm)')
% ylabel('test/model ratio')
