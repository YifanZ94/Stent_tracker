%% data set
clear; clc
% data = load('Motor_driven.txt');    
% trim_range = 125:1040;

% data = load('Manual_forward_IMU.txt');   
% trim_range = 260:950;

% data = load('static_on_hand_T11.5_IMU.txt');   
% trim_range = 1:150;

% data = load('Manual_hold_static.txt');   
% trim_range = 300:500;

% data = load('random_rotation_T22.0_IMU.txt');   
% trim_range = 220:950;

% data = load('manual_slow_2_IMU.txt');   
% trim_range = 200:800;

% data = load('servo_rotation_IMU.txt');
% trim_range = 570:1195;   % for servo

% data = load('6.83s_manual_IMU.txt');
% trim_range = 1:502;

% data = load('PosYaw_T28.55_IMU.txt');    
% trim_range = 1:size(data,1);


%%
Ts = 0.001;
init_range = 10;

acc_init = mean(data(1:init_range, 1:3), 1);
gyro_init = mean(data(1:init_range, 4:6), 1);

%  - acc_init'
acc = data(init_range+1:end, 1:3)';
gyro = (data(init_range+1:end, 4:6) - gyro_init)'*pi/180;

X = 1:size(acc,2);

% move_window = 10;
% acc = movmean(acc(X,:), move_window)';
% gyro = movmean(gyro(X, :), move_window)';

acc_mean = norm(acc_init);

L = size(acc,2);
dim = 1;

figure
plot((1:L)*Ts, acc')
xlabel('Time (s)')
ylabel('Acceleration (m/s2)')
legend('aX','aY','aZ')

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

Rot_by_Eulers = @(roll,pitch,yaw) [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)]* ...
                                [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)]*...
                                [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1];   % XYZ

Rot_by_Eulers = @(roll,pitch,yaw) [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1]* ...
            [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)]*...
            [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)];                     % ZYX
        
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

%% amplitude moving average
for i = 1:L
    acc_amp(i) = abs(norm(acc(:,i)) - acc_mean);
%     acc_amp(i) = norm(acc(:,i));
    gyro_amp(i) = norm(gyro(:,i));
end
move_window = 10;
acc_amp_ave = movmean(acc_amp, move_window);
gyro_amp_ave = movmean(gyro_amp, move_window);

a0 = 1.1*max(acc_amp_ave(1:100));
w0 = 1.1*max(gyro_amp_ave(1:100));

times = (X-X(1))/100;
for i = 1:L
    acc_is_static(i) = acc_amp_ave(i) < a0;
    gyro_is_static(i) = gyro_amp_ave(i) < w0;
end

% figure
% plot(times, acc_amp)
% xlabel('Time (S)')
% ylabel('Acc amp (m/s^2)')
% set(gca, 'Fontsize', 12)
% figure
% plot(times, gyro_amp)
% xlabel('Time (S)')
% ylabel('Gyro amp (rad/s)')
% set(gca, 'Fontsize', 12)

%% data moving std
H = 5;
for i = H:L
    acc_amp_std(i-H+1) = std(acc_amp_ave(i-H+1:i));
    gyro_amp_std(i-H+1) = std(gyro_amp_ave(i-H+1:i));
end

move_window = 10;
acc_std_ave = movmean(acc_amp_std, move_window);
gyro_std_ave = movmean(gyro_amp_std, move_window);

figure
plot([zeros(1,H), acc_amp_std])
xlabel('Time (S)')
ylabel('Acc std')
figure
plot([zeros(1,H), gyro_amp_std])
xlabel('Time (S)')
ylabel('Gyro std')

a_std_0 = 0.05;
w_std_0 = 0.01;

for i = 1:L-H
    acc_std_static(i+H) = acc_std_ave(i) < a_std_0;
    gyro_std_static(i+H) = gyro_std_ave(i) < w_std_0;
end

figure
subplot(121)
plot(gyro_std_static,'o')
title('gyro static')
subplot(122)
plot(acc_std_static,'o')
title('acc static')

% plot(times, acc_amp(X))
% xlabel('Time (S)')
% ylabel('Acc amp (m/s2)')
% figure
% plot(times, gyro_amp(X))
% xlabel('Time (S)')
% ylabel('Gyro amp (rads/s)')


%%
figure
plot(times, acc_amp_ave(X))
hold on
plot([0 X(end)-X(1)]/100,[a0 a0],'--','Linewidth',2)
% hold on
% plot([0 X(end)-X(1)]/100,[a1 a1],'--','Linewidth',2)
xlabel('Time (S)')
ylabel('Acc amp (m/s2)')
legend('Measurement', 'a0')
set(gca, 'Fontsize', 12)

w0 = 0.02;
figure
plot(times,gyro_amp(X))
hold on
plot([0 X(end)-X(1)]/100,[w0 w0],'--','Linewidth',2)
% hold on
% plot([0 X(end)-X(1)]/100,[w1 w1],'--','Linewidth',2)
xlabel('Time (S)')
ylabel('Gyro amp (rads/s)')
legend('Measurement', 'w0')
set(gca, 'Fontsize', 12)

%% acc correction
acc_diff_adap = [0;0;0];
acc_correct = Rot_by_Eulers(Euler(1), Euler(2), Euler(3))*acc(:,1);
i = 2;
conflict = 0;
acc_correct_amp = 0;
while i <= L        
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
%         quat(:,i) = q_byEuler(Euler(:,i));
    else
        Euler(:,i) = Euler(:,i-1);
    end

% % no correction
    Euler(:,i) = Euler_quat(:,i-1);

    acc_correct(:,i) = Rot_by_Eulers(Euler(1,i), Euler(2,i), Euler(3,i))*acc(:,i);
    acc_correct_amp(i) = norm(acc_correct(:,i));
    i = i+1;
end 

%%
figure
subplot(311)
plot(times, acc_correct(1,:)')
ylabel('ax (rad/s)')
title('corrected')
subplot(312)
plot(times, acc_correct(2,:)')
ylabel('ay (rad/s)')
subplot(313)
plot(times, acc_correct(3,:)')
ylabel('az (rad/s)')
xlabel('Time (S)')

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
% data_for_integ = acc(dim,X);

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
acc_diff_is_static = 1;

acc_state_partly = acc_is_static(X);

for i = 2:size(data_for_integ,2)
    
%     if acc_is_static(i) == 0
    if acc_state_partly(i) == 0
%         velocity(i) = max(velocity(i-1) + data_for_integ(i)*Ts, 0);
        velocity(i) = velocity(i-1) + data_for_integ(i)*Ts;
%         acc_diff_is_static(i) = 0;
    else
        velocity(i) = 0;
%         acc_diff_is_static(i) = 1;
    end
%     disp_delta(i) = (velocity(i) + velocity(i-1))*Ts/2;
end

displacement = cumtrapz(Ts, velocity);

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

%%
% InputCell{1} = [acc_amp; gyro_amp];
% save('test_manual','InputCell')