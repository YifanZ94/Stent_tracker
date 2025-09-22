% This program contains the valocity measurement obtained 
% based on the location measurement
clear; clc
%% dynamic equations
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

% quat_iteration_2 = @(omega,Ts)    eye(4)+ [0 -omega(1) -omega(2) -omega(3);
%                 omega(1) 0 omega(3) -omega(2);
%                 omega(2) -omega(3) 0 omega(1);
%                 omega(3) omega(2) -omega(1) 0]*Ts/2;

%% load all txt files
files = dir('*.txt') ;   
N = length(files) ;
InputCell = cell(N,1);
TargetCell = cell(N,1);

% 1:N
for test_num = 1:N
    clearvars -except files N InputCell TargetCell test_num quat_iteration Rot_by_quat Rot_by_Eulers Euler_by_quat Euler_by_acc
    thisfile = files(test_num).name ;
    data = load(thisfile);
    Ts = 0.03;
    init_range = 1;
    
    data = data(init_range+1:end, :) - data(1, :);
    acc = (data(:, 1:3)' + [0;0;9.8]);
    
%     data = data(init_range+1:end, :);
%     acc = data(:, 1:3)';
%     
    gyro = data(:, 4:6)'*pi/180;
    
    acc_3std = 3*norm(std(acc,0,2));
    gyro_3std = 3*norm(std(gyro,0,2));
    acc_static_3std = 0.03;  % m/s2
    acc_mean = 9.8;
    gyro_static_std = 0.05;  % rad
    dim = 1;
    L = size(acc,2);

for i = 1:L
    acc_amp(i) = norm(acc(:,i));
    acc_is_static(i) = acc_amp(i) < acc_mean+acc_static_3std && acc_amp(i) > acc_mean-acc_static_3std ;
    gyro_amp(i) = norm(gyro(:,i));
    gyro_is_static(i) = gyro_amp(i) < gyro_3std;
end

%% EKF initialize
Euler = Euler_by_acc(acc(:,1));
acc_diff = [0;0;0];
acc_correct(:,1) = Rot_by_Eulers(Euler(1), Euler(2), Euler(3))*acc(:,1);
quat(:,1) = q_byEuler(Euler);

%% acc correction
i = 2;
while i <= size(acc,2)
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
    acc_diff(:,i) = acc_correct(:,i) - acc_correct(:,i-1);
    acc_diff_norm(i) = norm(acc_diff(:,i));
    
    i = i+1;
end 

%% integration without zero velocity reset
% velocity_raw = 0;
% for i = 2:L
%     velocity_raw(i) = velocity_raw(i-1) + acc_correct(dim,i)*Ts;
% end
% displacement_raw = cumtrapz(Ts, velocity_raw);

%% integration acc with zero velocity reset
velocity = 0;
disp_delta = 0;
X = 100*acc_correct;
% X = 100*acc;

for i = 2:L
    if acc_diff_norm(i) >= 0.2
        velocity(i) = velocity(i-1) + X(dim,i)*Ts;
    else
        velocity(i) = 0;
    end
    disp_delta(i) = (velocity(i-1) + X(dim,i)*Ts/2)*Ts;
end
displacement = cumtrapz(Ts, velocity);   % unit of cm
disp_delta_sum = cumsum(disp_delta);

InputCell{test_num} = [X(dim,:); velocity; disp_delta; gyro];

end

save('DataSet_forTest.mat', 'InputCell')
disp('done')

% DataSet2_Disp_Eulers
% DataSet2_deltaDisp_Eulers