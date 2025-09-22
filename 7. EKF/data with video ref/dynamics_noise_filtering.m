%% data set
clear; clc
data = load('Test3_state.txt');
% data = load('Slow_40s_state.txt');

%%
Ts = 0.0145;
init_range = 10;

acc_init = mean(data(1:init_range, 1:3), 1);
gyro_init = mean(data(1:init_range, 4:6), 1);

acc = data(init_range+1:end, 1:3)';
gyro = (data(init_range+1:end, 4:6) - gyro_init)'*pi/180;

% move_window = 10;
% acc = movmean(acc(X,:), move_window)';
% gyro = movmean(gyro(X, :), move_window)';

acc_mean = norm(acc_init);
L = size(acc,2);
dim = 1;

% figure
% plot((1:L)*Ts, acc')
% xlabel('Time (s)')
% ylabel('Acceleration (m/s2)')
% legend('aX','aY','aZ')

%% additional vibration
L = size(acc,2);
vib_std = 0.2;
vib_X = vib_std*randn(1,L);
% acc_vib = acc + vib_X;
% acc = acc + vib_X;

% figure
% plot(cumsum(cumsum(acc(1,:)))')
% hold on
% plot(cumsum(cumsum(acc(1,:)))')

H = 20;
for i = H:L
    vib_move_std(i-H+1) = std(vib_X(1, i-H+1:i));
end

%% frequency spectrum
% Fs = 1/Ts;
% filter_data = ones(1,1236) + vib_X';
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

%% Low pass filter
Fs = 1/Ts;
f_cut = 0.2;
data_LP = lowpass(acc(1,:), f_cut, Fs, Steepness=0.95);
figure
plot(acc(1,:))
hold on
plot(data_LP)
xlabel('sampling number')
ylabel('acc in X (cm2/s)')
legend('raw data','low pass')

% range = 50:1000;
% acc = data_LP(range,:)';

%% Gaussian filter
f_window = 100;
g=fspecial('gaussian',[1 10], vib_std);
y_GF =conv(acc_correct(1,:), g);

subplot(131)
plot(acc2(1,:))
subplot(132)
plot(y_GF(f_window/2 :end-f_window/2))
subplot(133)
plot(acc(1,:))


%% dynamic model
quat_iteration = @(omega,Ts)    expm([0 -omega(1) -omega(2) -omega(3);
        omega(1) 0 omega(3) -omega(2);
        omega(2) -omega(3) 0 omega(1);
        omega(3) omega(2) -omega(1) 0]*Ts/2);
    
Rot_by_quat = @(q)   [q(1)^2+q(2)^2-q(3)^2-q(4)^2, 2*(q(2)*q(3)-q(1)*q(4)), 2*(q(1)*q(3)+q(2)*q(4));
         2*(q(2)*q(3)+q(1)*q(4)), q(1)^2-q(2)^2+q(3)^2-q(4)^2, 2*(q(3)*q(4)-q(1)*q(2));
         2*(q(2)*q(4)-q(1)*q(3)), 2*(q(1)*q(2)+q(3)*q(4)), q(1)^2-q(2)^2-q(3)^2+q(4)^2];

Rot_by_Eulers = @(roll,pitch,yaw) [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)]* ...
                                [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)]*...
                                [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1];

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

acc2 = acc;
for i = 1:3
    acc2(i,:) = movmean(acc(i,:), move_window);
end

acc_amp_ave = movmean(acc_amp, move_window);
gyro_amp_ave = movmean(gyro_amp, move_window);

acc_amp_ave = abs(acc_amp_ave - min(acc_amp_ave));
gyro_amp_ave = gyro_amp_ave - min(gyro_amp_ave);

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
    
%     if gyro_is_static(i) ~= acc_is_static(i)
%         conflict(i) = 1;
%     end
%     plot(conflict)
        
%  threshold by gyro amp
    if gyro_is_static(i) == 0
        Euler(:,i) = Euler_quat(:,i);
    elseif acc_is_static(i) == 1
        Euler(:,i) = Euler_acc(:,i);
        quat(:,i) = q_byEuler(Euler(:,i));
    else
        Euler(:,i) = Euler(:,i-1);
    end

% % no correction
%     Euler(:,i) = Euler(:,i-1);

    acc_correct(:,i) = Rot_by_Eulers(Euler(1,i), Euler(2,i), Euler(3,i))*acc(:,i);
    acc_correct_amp(i) = norm(acc_correct(:,i));
    i = i+1;
    
end 

%% corrected acc
% figure
% subplot(311)
% plot(times, acc_correct(1,:)')
% ylabel('ax (rad/s)')
% title('corrected')
% subplot(312)
% plot(times, acc_correct(2,:)')
% ylabel('ay (rad/s)')
% subplot(313)
% plot(times, acc_correct(3,:)')
% ylabel('az (rad/s)')
% xlabel('Time (S)')

%% data std
H = 5;
for i = H:L
    acc_amp_std(i-H+1) = std(acc_correct(1, i-H+1:i));
    gyro_amp_std(i-H+1) = std(gyro_amp_ave(i-H+1:i));
end

a_std_0 = 0.06 + vib_std;
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
% data_for_integ = data_LP(range, dim)+ 0.82;
% times = Ts*(1:size(data_for_integ,2));
% velocity_raw = cumtrapz(Ts, data_for_integ);
% displacement_raw = cumtrapz(Ts, velocity_raw);
% figure
% subplot(311)
% plot(times,data_for_integ)
% ylabel('acceleration (m/s2)')
% subplot(312)
% plot(times,velocity_raw)
% ylabel('velocity (m/s)')
% subplot(313)
% plot(times,displacement_raw)
% ylabel('Displacement (m)')

%% integration acc with zero velocity reset
% X = data_LP(:,dim);
data_for_integ = acc_correct(dim,:);
times = Ts*(1:size(data_for_integ,2));
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