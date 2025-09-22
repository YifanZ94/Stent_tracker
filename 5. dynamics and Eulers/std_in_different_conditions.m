%% data set
clear; clc
% data = load('Motor_driven.txt');    
% X = 125:1040;

% data = load('Manual_forward_IMU.txt');   
% X = 356:975;

% data = load('static_on_hand_T11.5_IMU.txt');   %---  acc_amp std 0.02, gyro_amp_std 0.003
% X = 1:150;

% data = load('Manual_hold_static.txt');   %---  acc_amp std 0.06, gyro_amp_std 0.06
% X = 71:888;

% data = load('manual_slow_IMU.txt');   
% X = 220:950;

% data = load('static_51.21.txt');   %---  acc_amp std 0.005, gyro_amp std 0.002
% X = 1:size(data,1);

a_std_0 = 0.005;
a_std_1 = 0.02;
%%
Ts = 0.01;
init_range = 10;

acc_init = mean(data(1:init_range, 1:3), 1);
gyro_init = mean(data(1:init_range, 4:6), 1);

%  - acc_init'
acc = data(init_range+1:end, 1:3);
gyro = (data(init_range+1:end, 4:6) - gyro_init)*pi/180;

% X = 1:size(acc,1);

move_window = 10;
acc = movmean(acc(X,:), move_window);
gyro = movmean(gyro(X, :), move_window);

plot(acc)

%% std of each axis
L = size(acc,1);
acc_init_norm = norm(acc_init);
for i = 1:L
    acc_amp(i) = abs(norm(acc(i,:)) - acc_init_norm);
    gyro_amp(i) = norm(gyro(i,:));
end

acc_std = std(acc_amp);
gyro_std = std(gyro_amp);

w1 = 0.04;
w0 = 0.005;

figure
plot(acc_amp)
hold on
plot(3*w1*ones(1,L), '--')

% acc_axes_std = std(acc_std,0,2);
% gyro_axes_std = std(gyro_std,0,2);
% figure
% subplot(211)
% plot(acc_axes_std)
% subplot(212)
% plot(gyro_axes_std)
