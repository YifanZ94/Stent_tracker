%% data set
clear; clc
data = load('t2_IMU.txt');  % Ts = 0.045
data = data';

% data = load('Slow_40s_state.txt');
% tempo = data(:,1);
% data(:,1) = data(:,2);
% data(:,2) = tempo;
% 
% tempo = data(:,4);
% data(:,4) = data(:,5);
% data(:,5) = tempo;

%%
% acc = load('t_acc.txt');
% gyro = load('t_gyro.txt');
% data = [acc; 100*gyro]';

%%
Ts = 0.01;
init_range = 10;

acc_init = mean(data(1:init_range, 1:3), 1);
gyro_init = mean(data(1:init_range, 4:6), 1);

acc = data(init_range+1:end, 1:3)';
gyro = (data(init_range+1:end, 4:6) - gyro_init)';
mag = data(init_range+1:end, 7:9)';

acc_mean = norm(acc_init);

X = 1:size(acc,2);
L = size(acc,2);
dim = 1;

% figure
% plot((1:L)*Ts, acc')
% xlabel('Time (s)')
% ylabel('Acceleration (m/s2)')
% legend('aX','aY','aZ')

%% additional vibration
% L = size(acc,2);
% vib_std = 0.3;
% vib_X = vib_std*rand(1,L) - vib_std/2;
% acc = acc + vib_X;

%%
% Fs = 1/Ts;
% filter_data = acc';
% Y = fft(filter_data);
% P2 = abs(Y/L);
% P1 = P2(1:floor(L/2)+1,:);
% P1(2:end-1,:) = 2*P1(2:end-1,:);
% f = Fs*(0:(L/2))/L;
% figure
% plot(f,P1)
% legend('x','y','z')
% title('Single-Sided Amplitude Spectrum')
% xlabel('f (Hz)')
% ylabel('|P1(f)|')
% 
% f_cut = 0.1;
% data_LP = lowpass(filter_data,f_cut,Fs,Steepness=0.95);
% figure
% plot(filter_data(:,1))
% hold on
% plot(data_LP(:,1))
% xlabel('sampling number')
% ylabel('acc in X (cm2/s)')
% legend('raw data','low pass')
% acc = data_LP';

%% dynamic model
quat_iteration = @(omega,Ts)    expm([0 -omega(1) -omega(2) -omega(3);
        omega(1) 0 omega(3) -omega(2);
        omega(2) -omega(3) 0 omega(1);
        omega(3) omega(2) -omega(1) 0]*Ts/2);
    
Rot_by_quat = @(q)   [q(1)^2+q(2)^2-q(3)^2-q(4)^2, 2*(q(2)*q(3)-q(1)*q(4)), 2*(q(1)*q(3)+q(2)*q(4));
         2*(q(2)*q(3)+q(1)*q(4)), q(1)^2-q(2)^2+q(3)^2-q(4)^2, 2*(q(3)*q(4)-q(1)*q(2));
         2*(q(2)*q(4)-q(1)*q(3)), 2*(q(1)*q(2)+q(3)*q(4)), q(1)^2-q(2)^2-q(3)^2+q(4)^2];

% Rot_by_Eulers = @(roll,pitch,yaw) [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)]* ...
%                                 [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)]*...
%                                 [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1];            % XYZ
                            
Rot_by_Eulers = @(roll,pitch,yaw) [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1]* ...
                                  [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)]* ...
                                  [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)];    % ZYX
                              

Euler_by_quat = @(q) [atan2(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2))
                      asin(2*(q(1)*q(3)- q(4)*q(2)));  
                      atan2(2*(q(1)*q(4)+q(2)*q(3)), 1-2*(q(3)^2+q(4)^2));];  % X, Z reversed
                  
Euler_by_acc = @(a) [atan2(a(2),a(3));
                    atan(-a(1)/sqrt(a(2)^2 + a(3)^2));
                    0];

quat_iteration_2 = @(omega,Ts)    eye(4)+ [0 -omega(1) -omega(2) -omega(3);
                omega(1) 0 omega(3) -omega(2);
                omega(2) -omega(3) 0 omega(1);
                omega(3) omega(2) -omega(1) 0]*Ts/2;

% Matrix_exp = @(omega,ord) [0, -omega(1), -omega(2), -omega(3);
%                 omega(1), 0, omega(3), -omega(2);
%                 omega(2), -omega(3), 0, omega(1);
%                 omega(3), omega(2), -omega(1), 0]^ord/factorial(ord);


%% EKF initialize
Euler = Euler_by_acc(acc_init);
acc_correct(:,1) = Rot_by_Eulers(Euler(1), Euler(2), Euler(3))*acc_init';
quat(:,1) = q_byEuler(Euler);
% Euler_quat = Euler_by_quat(quat)*180/pi; 
Euler_quat = Euler;

%% check the threshold and the dynamic range
for i = 1:L
    acc_amp(i) = abs(norm(acc(:,i)) - acc_mean);
%     acc_amp(i) = norm(acc(:,i));
    gyro_amp(i) = norm(gyro(:,i));
end

% acc_diff_norm(1) = 0;
% for i = 2:L
%     acc_diff(:,i) = acc(:,i) - acc(:,i-1);
%     acc_diff_norm(i) = norm(acc_diff(:,i));
% end
% a_diff_0 = 0.2;

%% moving average
a0 = 0.02;
w0 = 0.02;
a1 = 0.05;
w1 = 0.05;

move_window = 10;
acc_amp_ave = movmean(acc_amp, move_window);
gyro_amp_ave = movmean(gyro_amp, move_window);

acc_amp_ave = abs(acc_amp_ave - min(acc_amp_ave));
gyro_amp_ave = gyro_amp_ave - min(gyro_amp_ave);

times = Ts*X;

%%
% figure
% plot(times, acc_amp_ave(X))
% hold on
% plot([0 X(end)-X(1)]/100,[a0 a0],'--','Linewidth',2)
% hold on
% plot([0 X(end)-X(1)]/100,[a1 a1],'--','Linewidth',2)
% xlabel('Time (S)')
% ylabel('Acc amp (m/s2)')
% legend('Measurement', 'a0', 'a1')
% 
% figure
% plot(times,gyro_amp(X))
% hold on
% plot([0 X(end)-X(1)]/100,[w0 w0],'--','Linewidth',2)
% hold on
% plot([0 X(end)-X(1)]/100,[w1 w1],'--','Linewidth',2)
% xlabel('Time (S)')
% ylabel('Gyro amp (rads/s)')
% legend('Measurement', 'w0', 'w1')

%%
for i = 1:L
    acc_is_static(i) = acc_amp_ave(i) < a0;
    gyro_is_static(i) = gyro_amp_ave(i) < w0;

%     acc_is_static(i) = acc_diff_norm(i) < a_diff_0;
%     gyro_is_static(i) = gyro_amp(i) < a_diff_0;
end

% acc_is_static(1:H) = 0;
% gyro_is_static(1:H) = 0;
% for i = 1:L-H
%     acc_is_static(i+H) = acc_amp_std(i) < a_std_0;
%     gyro_is_static(i+H) = gyro_amp_std(i) < w_std_0;
% end

% figure
% subplot(121)
% plot(gyro_is_static,'o')
% title('gyro static')
% subplot(122)
% plot(acc_is_static,'o')
% title('acc static')


%% acc correction
acc_diff_adap = [0;0;0];
acc_correct = Rot_by_Eulers(Euler(1), Euler(2), Euler(3))*acc(:,1);
i = 2;
conflict = 0;
acc_correct_amp = 0;

while i <= L    
%     acc_diff_adap(:,i) = Rot_by_Eulers(Euler(1,i-1), Euler(2,i-1), Euler(3,i-1))*acc(:,i) - acc_correct(:,i-1);
%     acc_diff_adap_norm(i) = norm(acc_diff_adap(:,i));
%     acc_is_static(i) = acc_diff_adap_norm(i) < a1;

    Euler_acc(:,i) = Euler_by_acc(acc(:,i));
    quat(:,i) = quat_iteration(gyro(:,i),Ts)*quat(:,i-1);
    Euler_quat(:,i) = Euler_by_quat(quat(:,i)); 
    Euler_acc(3,i) = Euler_quat(3,i);
    
%     if gyro_is_static(i) ~= acc_is_static(i)
%         conflict(i) = 1;
%     end
%     plot(conflict)
        
%  threshold by gyro amp
%     if gyro_is_static(i) == 0
%         Euler(:,i) = Euler_quat(:,i);
%     elseif acc_is_static(i) == 1
%         Euler(:,i) = Euler_acc(:,i);
%         quat(:,i) = q_byEuler(Euler(:,i));
%     else
%         Euler(:,i) = Euler(:,i-1);
%     end

% % no correction
    Euler(:,i) = Euler_quat(:,i);

    acc_correct(:,i) = Rot_by_Eulers(Euler(1,i), Euler(2,i), Euler(3,i))*acc(:,i);
    acc_correct_amp(i) = norm(acc_correct(:,i));
    i = i+1;
    
end 

%% corrected acc
figure
subplot(311)
plot(times, acc_correct(1,:)')
hold on
plot(times, acc(1,:)')
ylabel('ax (rad/s)')
title('corrected')
subplot(312)
plot(times, acc_correct(2,:)')
hold on
plot(times, acc(2,:)')
ylabel('ay (rad/s)')
subplot(313)
plot(times, acc_correct(3,:)')
hold on
plot(times, acc(3,:)')
ylabel('az (rad/s)')
xlabel('Time (S)')
legend('corrected', 'raw')

%% data std
H = 5;
for i = H:L
    acc_amp_std(i-H+1) = std(acc_correct(1, i-H+1:i));
    gyro_amp_std(i-H+1) = std(gyro_amp_ave(i-H+1:i));
end

a_std_0 = 0.06;
w_std_0 = 0.005;

acc_amp_std = [zeros(1,H-1), acc_amp_std];
gyro_amp_std = [zeros(1,H-1), gyro_amp_std];

% figure
% plot(acc_amp_std)
% hold on
% plot([0 L], [a_std_0 a_std_0],'--','Linewidth',2)
% xlabel('Time (S)')
% ylabel('Acc std')
% figure
% plot(gyro_amp_std)
% hold on
% plot([0 L],[w_std_0 w_std_0],'--','Linewidth',2)
% xlabel('Time (S)')
% ylabel('Gyro std')


%% Eulers
% figure
% subplot(131)
% plot(times, Euler_quat'*180/pi)
% ylabel('Eulers by Gyro (degree)')
% xlabel('Time (s)')
% legend('roll','pitch','yaw')
% subplot(132)
% plot(times, Euler_acc'*180/pi)
% % plot((Euler_acc(2,:)-Euler_acc(2,1))'*180/pi)
% ylabel('Euler acc (degree)')
% subplot(133)
% plot(times, Euler'*180/pi)
% ylabel('Eulers Fused (degree)')

%% integration without zero velocity reset
data_for_integ = acc_correct(dim,X);
velocity_raw = cumtrapz(Ts, data_for_integ);
displacement_raw = cumtrapz(Ts, velocity_raw);
figure
subplot(311)
plot(times,acc(1,X))
ylabel('acceleration (m/s2)')
subplot(312)
plot(times,velocity_raw)
ylabel('velocity (m/s)')
subplot(313)
plot(times,displacement_raw)
ylabel('Displacement (m)')

%% integration acc with zero velocity reset
% X = data_LP(:,dim);
data_for_integ = acc_correct(dim,X);
velocity = 0;
acc_trans_static = 0;

for i = 2:size(data_for_integ,2)
    
    acc_trans_static(i) = acc_amp_std(i) < a_std_0;
%     if acc_is_static(i) == 0
    if acc_trans_static(i) == 0
%         velocity(i) = max(velocity(i-1) + data_for_integ(i)*Ts, 0);
        velocity(i) = velocity(i-1) + data_for_integ(i)*Ts;
%         acc_diff_is_static(i) = 0;
    else
        velocity(i) = 0;
%         acc_diff_is_static(i) = 1;
    end
    disp_delta(i) = (velocity(i) + velocity(i-1))*Ts/2;
end

displacement = cumtrapz(Ts, velocity);
% displacement2 = cumsum(disp_delta);

figure
subplot(311)
plot(times, data_for_integ)
% legend('Euler and v reset','raw')
xlabel('time (s)')
ylabel('Acc (m/s2)')
subplot(312)
plot(times, velocity)
% legend('Euler correct','raw')
xlabel('time (s)')
ylabel('velocity (m/s)')
subplot(313)
plot(times, displacement)
% legend('Euler correct','raw')
xlabel('time (s)')
ylabel('displacement (m)')

%% slow or fast quantify
% acc_is_static
% acc_trans_static
% range = 1:1338;
range = 1:size(acc_correct,2);

k = find(acc_trans_static(range)==1);
dyn_accX = acc_correct(1,range);
dyn_accX(k) = [];
dyn_accX_mean = mean(abs(dyn_accX));
dyn_accX_max = max(abs(dyn_accX));

% figure
% plot(dyn_accX)
%%
figure
plot(times(1:end-165), Euler(:, 166:end)')
xlabel('Time (s)')
ylabel('Eulers (rads)')
legend('roll','pitch','yaw')
set(gca, 'Fontsize', 12)

%%
% InputCell{1} = [acc_correct(dim,X); velocity; disp_delta; acc_trans_static ; times; gyro_amp];
% save('test_1','InputCell')