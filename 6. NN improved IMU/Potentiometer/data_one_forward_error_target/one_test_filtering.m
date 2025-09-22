% This program contains the valocity measurement obtained 
% based on the location measurement
clear; clc

data = load('Test1.txt');
% T_all = 7.37;
% Ts = T_all/size(data,1);
Ts = 0.031;
init_range = 1;
acc = data(init_range+1:end, 1:3)';
gyro = data(init_range+1:end, 4:6)'*pi/180;

acc_3std = 3*norm(std(acc,0,2));
gyro_3std = 3*norm(std(gyro,0,2));

acc_static_3std = 0.03;  % m/s2
acc_mean = 9.8;
gyro_static_std = 0.05;  % rad

L = size(acc,2);
dim = 1;

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
Euler = Euler_by_acc(acc(:,1));
gyro_amp_diff = 0;
acc_correct(:,1) = Rot_by_Eulers(Euler(1), Euler(2), Euler(3))*acc(:,1);
quat(:,1) = q_byEuler(Euler);

%% check the threshold and the dynamic range
for i = 1:L
    acc_amp(i) = norm(acc(:,i));
    acc_is_static(i) = acc_amp(i) < acc_mean+acc_static_3std && acc_amp(i) > acc_mean-acc_static_3std ;
    gyro_amp(i) = norm(gyro(:,i));
    gyro_is_static(i) = gyro_amp(i) < gyro_3std;
end

figure(1)
scatter(1:L, acc_is_static)
hold on
scatter(1:L, gyro_is_static)
legend('acc is static','gyro is static')
hold on

%% acc correction
acc_diff = [0;0;0];

for i = 2:L
    acc_diff(:,i) = acc(:,i) - acc(:,i-1);
    acc_diff_norm(i) = norm(acc_diff(:,i));
    
    Euler_acc(:,i) = Euler_by_acc(acc(:,i));
    quat(:,i) = quat_iteration(gyro(:,i),Ts)*quat(:,i-1);
    Euler_quat(:,i) = Euler_by_quat(quat(:,i)); 

%  threshold by gyro amp
    if gyro_is_static(i) == 0
        Euler(:,i) = Euler_quat(:,i);
    elseif acc_is_static(i) == 1
        Euler(:,i) = Euler_acc(:,i);
        quat(:,i) = q_byEuler(Euler(:,i));
    else
        Euler(:,i) = Euler(:,i-1);
    end
    acc_correct(:,i) = Rot_by_Eulers(Euler(1,i), Euler(2,i), Euler(3,i))*acc(:,i);
    
end 

%% Effect of Eulers
velocity_raw = cumtrapz(Ts, acc(dim,:) - data(1,dim));
displacement_raw = cumtrapz(Ts, velocity_raw);
velocity_EulerCorrect = cumtrapz(Ts, acc_correct(dim,:));
displacement_EulerCorrect = cumtrapz(Ts, velocity_EulerCorrect);
figure
subplot(221)
plot(Euler')
legend('roll','pitch','yaw')
subplot(222)
plot(acc(dim,:)')
hold on
plot(acc_correct(dim,:)')
legend('raw acc','corrected acc')
subplot(223)
plot(velocity_raw)
hold on
plot(velocity_EulerCorrect)
ylabel('velocity')
subplot(224)
plot(displacement_raw)
hold on
plot(displacement_EulerCorrect)
ylabel('disp')

%% integration acc with zero velocity reset
X = 100*acc_correct;
% X = acc - acc(:,1);
velocity(i) = 0;

for i = 2:L
    if acc_diff_norm(i) >= 0.2
        velocity(i) = velocity(i-1) + X(dim,i)*Ts;
        acc_diff_is_static(i) = 0;
    else
        velocity(i) = 0;
        acc_diff_is_static(i) = 1;
    end
    disp_delta(i) = (velocity(i-1) + X(dim,i)*Ts/2)*Ts;
end
displacement = cumtrapz(Ts, velocity);

displacement_delta = cumsum(disp_delta);

figure
subplot(121)
Xs = (1:size(acc,2))*Ts;
plot(velocity')
xlabel('time (s)')
ylabel('V (cm/s)')
title('with velocity reset')
subplot(122)
plot(displacement')
xlabel('time (s)')
ylabel('disp (cm)')

%% ref
V = data(init_range+1:end, 7);
L_max = 26;
V0 = 1.65;
D0 = -(V(1) - V0)/(V0/L_max);
Dist = -(V - V0)/(V0/L_max) - D0;
figure
plot(V)
hold on
plot(Dist)
title('voltage and dist')