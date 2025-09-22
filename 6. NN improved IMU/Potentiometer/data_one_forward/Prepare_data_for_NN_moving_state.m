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
    Ts = 0.0145;
    init_range = 10;
    acc_init = mean(data(1:init_range, 1:3), 1);
    gyro_init = mean(data(1:init_range, 4:6), 1);
    acc = data(init_range+1:end, 1:3)';
    gyro = (data(init_range+1:end, 4:6) - gyro_init)'*pi/180;
    acc_mean = norm(acc_init);
    
    X = 1:size(acc,2);
    L = size(acc,2);
    dim = 1;
   
    Euler = Euler_by_acc(acc_init);
    acc_correct(:,1) = Rot_by_Eulers(Euler(1), Euler(2), Euler(3))*acc_init';
    quat(:,1) = q_byEuler(Euler);
    Euler_quat = Euler;
    
    for i = 1:L
        acc_amp(i) = abs(norm(acc(:,i)) - acc_mean);
        gyro_amp(i) = norm(gyro(:,i));
    end

    acc_diff_norm(1) = 0;
    for i = 2:L
        acc_diff(:,i) = acc(:,i) - acc(:,i-1);
        acc_diff_norm(i) = norm(acc_diff(:,i));
    end
    a0 = 0.02;
    w0 = 0.02;
    a1 = 0.05;
    w1 = 0.05;
    move_window = 10;
    acc_amp_ave = movmean(acc_amp, move_window);
    gyro_amp_ave = movmean(gyro_amp, move_window);
    acc_amp_ave = abs(acc_amp_ave - min(acc_amp_ave));
    gyro_amp_ave = gyro_amp_ave - min(gyro_amp_ave);
    
    for i = 1:L
        acc_is_static(i) = acc_amp_ave(i) < a0;
        gyro_is_static(i) = gyro_amp_ave(i) < w0;
    end
    
    acc_correct = Rot_by_Eulers(Euler(1), Euler(2), Euler(3))*acc(:,1);
    i = 2;
    conflict = 0;
    acc_correct_amp = 0;

    while i <= L    
        Euler_acc(:,i) = Euler_by_acc(acc(:,i));
        quat(:,i) = quat_iteration(gyro(:,i),Ts)*quat(:,i-1);
        Euler_quat(:,i) = Euler_by_quat(quat(:,i)); 
        if gyro_is_static(i) == 0
            Euler(:,i) = Euler_quat(:,i);
        elseif acc_is_static(i) == 1
            Euler(:,i) = Euler_acc(:,i);
        else
            Euler(:,i) = Euler(:,i-1);
        end
        acc_correct(:,i) = Rot_by_Eulers(Euler(1,i), Euler(2,i), Euler(3,i))*acc(:,i);
%         acc_correct_amp(i) = abs(norm(acc_correct(:,i)) - acc_mean);
        i = i+1;
    end 

    H = 5;
    acc_amp_std = zeros(1,H);
    gyro_amp_std = zeros(1,H);
    for i = H:L
        acc_amp_std(i+1) = std(acc_correct(1, i-H+1:i));
        gyro_amp_std(i+1) = std(gyro_amp_ave(i-H+1:i));
    end

    a_std_0 = 0.06;
    w_std_0 = 0.005;
    
    data_for_integ = acc_correct(dim,X);
    velocity = 0;
    acc_trans_static = 0;

    for i = 2:size(data_for_integ,2)
        acc_trans_static(i) = acc_amp_std(i) < a_std_0;
        if acc_trans_static(i) == 0
            velocity(i) = velocity(i-1) + data_for_integ(i)*Ts;
        else
            velocity(i) = 0;
        end

        velocity(i) = max(velocity(i), 0);

        disp_delta(i) = (velocity(i) + velocity(i-1))*Ts/2;
    end

%% displacement ref equal to IMU disp
V = data(init_range+1:end,7);
L_max = 0.26;  % fast data
V0 = 1.65;

% L_max = 0.23;  % slow data
% V0 = 3.7; 

D0 = -(V(1) - V0)/(V0/L_max);
Dist_ref = (-(V - V0)/(V0/L_max) - D0)';

for i = 2:L
    Dist_delta_ref(i) = -(V(i) - V(i-1))/(V0/L_max);
end

Dist_delta_smooth = movmean(Dist_delta_ref,15);
times = (1:size(Dist_delta_smooth,2))*Ts;

% InputCell{test_num} = [X(dim,:); velocity; displacement; gyro];
% TargetCell{test_num} = Dist_ref;

InputCell{test_num} = [acc_correct(dim,X); velocity; disp_delta; acc_trans_static ; ones(1,size(Dist_delta_smooth,2))*Ts; gyro_amp; acc_amp];
TargetCell{test_num} = Dist_delta_smooth;

end

% figure
% plot(times, Dist_delta_smooth)
% xlabel('Time (s)')
% ylabel('\Delta S in X (cm)')
% 
% figure
% plot(times, acc_correct)
% xlabel('Time (s)')
% ylabel('Corrected acc (m/s^2)')
% legend('a_X','a_Y','a_Z')
% 
% figure
% plot(times, velocity)
% xlabel('Time (s)')
% ylabel('Filtered V in X (cm/s)')
% 
% figure
% plot(times, gyro')
% xlabel('Time (s)')
% ylabel('Gyro (rads/s)')

save('DataSet_train_fast_0.03.mat', 'InputCell', 'TargetCell')
disp('done')

% DataSet2_Disp_Eulers
% DataSet2_deltaDisp_Eulers