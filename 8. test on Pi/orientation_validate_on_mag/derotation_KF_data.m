%% data set
clear; clc
Eulers = load('rot1_Euler.txt');
IMU = load('rot1_IMU.txt');
acc = IMU(1:3,:);
mag = IMU(7:9,:);
mag(:,3) = -mag(:,3);

%% bias remove
% mag = mag - [0.3; 0.05; -0.07];

%% dynamic model
quat_iteration = @(omega,Ts)    expm([0 -omega(1) -omega(2) -omega(3);
        omega(1) 0 omega(3) -omega(2);
        omega(2) -omega(3) 0 omega(1);
        omega(3) omega(2) -omega(1) 0]*Ts/2);
    
Rot_by_quat = @(q)   [q(1)^2+q(2)^2-q(3)^2-q(4)^2, 2*(q(2)*q(3)-q(1)*q(4)), 2*(q(1)*q(3)+q(2)*q(4));
         2*(q(2)*q(3)+q(1)*q(4)), q(1)^2-q(2)^2+q(3)^2-q(4)^2, 2*(q(3)*q(4)-q(1)*q(2));
         2*(q(2)*q(4)-q(1)*q(3)), 2*(q(1)*q(2)+q(3)*q(4)), q(1)^2-q(2)^2-q(3)^2+q(4)^2];

Rot_by_Eulers_XYZ = @(roll,pitch,yaw) [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)]* ...
                                [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)]*...
                                [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1];
                            
Rot_by_Eulers_ZYX = @(roll,pitch,yaw) [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1]* ...
            [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)]*...
            [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)];

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

Rx = @(roll) [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)];
Ry = @(pitch) [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)];
Rz = @(yaw) [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1];

% Eulers = Eulers*2;
% Eulers(3,:) = 1.5*(Eulers(3,:) - Eulers(3,1));
%%
ref_vec = [1;2;3];
data_ref = mag;

for i = 2:size(data_ref,2)
    mag_amp(i) = norm(mag(:,i));
    acc_amp(i) = norm(acc(:,i));
    
    Euler_acc(:,i) = Euler_by_acc(acc(:,i));
%     Euler_acc(3,i) = Eulers(3,i);
end

E = Euler_acc;

for i = 2:size(data_ref,2)   
    mag_correct_1(:,i) = Rot_by_Eulers_ZYX(E(1,i), E(2,i), E(3,i))*data_ref(:,i);
    mag_correct_2(:,i) = Rot_by_Eulers_ZYX(-E(1,i), -E(2,i), -E(3,i))*data_ref(:,i);
%     mag_correct_2(:,i) = Rot_by_Eulers_XYZ(E(1,i), E(2,i), E(3,i))*data_ref(:,i);
    
%     mag_correct_1(:,i) = Ry(-E(2,i))*Rx(-E(1,i))*data_ref(:,i);
%     mag_correct_1(:,i) = Rz(E(3,i))*mag_correct_1(:,i);
    
%     mag_correct_1(:,i) = Rz(Eulers(3,i)*1.5)*data_ref(:,i);    % r
%     mag_correct_2(:,i) = Rz(-Eulers(3,i))*data_ref(:,i);
end

figure
subplot(131)
plot(data_ref(1,:))
hold on
plot(mag_correct_1(1,:))
ylabel('Bx')
hold on
% plot(mag_correct_2(1,:))
legend('raw', 'R ZYX', 'R XYZ')

subplot(132)
plot(data_ref(2,:))
hold on
plot(mag_correct_1(2,:))
ylabel('By')
hold on
% plot(mag_correct_2(2,:))

subplot(133)
plot(data_ref(3,:))
hold on
plot(mag_correct_1(3,:))
ylabel('Bz')
hold on
% plot(mag_correct_2(3,:))

figure
subplot(121)
plot(Eulers')
% subplot(122)
% plot(Euler_acc')