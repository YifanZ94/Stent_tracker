clear; clc
data = load('In_air_T11.1_IMU.txt');
acc = (data(:,1:3) - mean(data(1:20,1:3)));
T_all = 11.1;
L = size(acc,1);
Ts = T_all/L;
Fs = 1/Ts;

%%  FFT
Fs = 1/Ts;
Y = fft(acc);
P2 = abs(Y/L);
P1 = P2(1:floor(L/2)+1,:);
P1(2:end-1,:) = 2*P1(2:end-1,:);
f = Fs*(0:(L/2))/L;
figure
plot(f,P1)
legend('x','y','z')
title('Single-Sided Amplitude Spectrum')
xlabel('f (Hz)')
ylabel('|P1(f)|')

%%  lowpass
f_cut = 2;
data_LP = lowpass(acc,f_cut,Fs);
figure
plot(acc(:,1))
hold on
plot(data_LP(:,1))
xlabel('sampling number')
ylabel('acc in X (cm2/s)')
legend('raw data','low pass')
% title('low pass data')

%%  bandpass
band_cut = [0.1 2];
data_BP = bandpass(acc,band_cut,Fs);
% figure
% plot(data_BP)
% title('band pass data')
%% rotation matrix
% angle = cumtrapz(Ts,gyro*180/pi);
% figure
% plot(angle)
% title('angle (degree)')

% Rz = @(theta) [cos(theta),-sin(theta),0; sin(theta),cos(theta),0; 0,0,1]; 
% 
% for i = 1:size(data,1)
%     acc_R(i,:) = (Rz(angle(i))\data_LP(i,:)')';
% end

%% post time plot
velocity = cumtrapz(Ts,acc);
displacement = cumtrapz(Ts,velocity);

velocity_LP = cumtrapz(Ts,data_LP);
displacement_LP = cumtrapz(Ts,velocity_LP);

figure
plot(displacement(:,1))
% legend('X','Y')
hold on
plot(displacement_LP(:,1))
xlabel('sampling number')
ylabel('integrated location')
legend('X','X-LP')

%% real time plot
% figure
% xlim([min(displacement(:,1))-70 max(displacement(:,1))+50])
% ylim([min(displacement(:,2))-70 max(displacement(:,2))+50])
% xlabel('X location (cm)')
% ylabel('Y location (cm)')
% hold on
% for i = 1:size(acc,1)
%     plot(displacement(i,1),displacement(i,2),'ro')
%     hold on
%     pause(0.02)
% end

