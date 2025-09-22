clear; clc
%%
data = load('Intermittent_IMU.txt');
L = size(data,1);
Ts = 0.015;
Fs = 1/Ts;
data_index = 7:9;
acc = data(:,1:3);
gyro = data(:,4:6);
mag = data(:,7:9);

for i = 1:L
    mag_amp(i) = norm(mag(i,:));
    acc_amp(i) = norm(acc(i,:));
    gyro_amp(i) = norm(gyro(i,:));
end
% plot((1:L)*Ts, mag_amp)
plot(mag)
xlabel('Time (S)')
ylabel('Mag (G)')

%% actual disturbance
% data2 = load('disturb_IMU.txt');
% M_d = data2(1:L,7:9);
% M_d = M_d - mean(M_d(1:20,:), 1);
% 
% for i = 1:L
%     mag_amp_d(i) = norm(M_d(i,:));
% end
% plot(mag_amp_d)

%% simulated disturbance with high frequency
M_d = zeros(1,size(mag_amp,2));
disturb_periods = [200:400, 600:800, 1000:1200];
f_d = 10;
M_d(disturb_periods) = sin(2*pi*f_d*disturb_periods*Ts);

for i = 1:L
    M_d(i) = (2*rand-1)*M_d(i);
    mag_amp_d(i) = norm(M_d(:,i));
end

plot(mag_amp_d)

%%
% X = (mag_amp(1:size(mag_amp_d,2)) + mag_amp_d)';
% X = mag_amp_d';
trim_range = 181:814;
L = size(trim_range,2);
plot(X)
xlabel('Time (S)')
ylabel('Mag (G)')

%%  FFT
X = [acc_amp', gyro_amp', mag_amp'];
Y = fft(X);
P2 = abs(Y/L);
P1 = P2(1:floor(L/2)+1,:);
P1(2:end-1,:) = 2*P1(2:end-1,:);
f = Fs*(0:(L/2))/L;
plot(f,P1)
% legend('x','y','z')
title('Single-Sided Amplitude Spectrum')
% ylim([0 0.2])
xlabel('f (Hz)')
ylabel('|P1(f)|')

%%  lowpass
f_cut = 0.1;
Y_LP = lowpass(X,f_cut,Fs,Steepness=0.99);
figure
lowpass(X,f_cut,Fs)

figure
plot(Y_LP)
hold on
plot(mag_amp)
%%  bandpass
% band_cut = [5 20];
% Y_LP = bandpass(X,band_cut,Fs);
% bandpass(X,band_cut,Fs)

