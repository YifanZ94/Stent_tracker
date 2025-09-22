% This program contains the valocity measurement obtained 
% based on the location measurement
clear; clc
data = load('Manual_IMU.txt');
initial_index = 20;
data = data(initial_index+1:end,:) - data(1,:);
data(93,:) = [];
g = [0;0;9.8];
acc = data(:,1:3)' + g;
gyro = data(:,4:6)'*pi/180;
magnetic = data(:,7:9)';
Ts = 0.03;

load('Trained_NN_oneMove.mat')

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

%% normalize magnetic field
load('Square_X40_Y20_Z15_reso0.2_T.mat')
BxR = reshape(BX,[], 1); 
ByR = reshape(BY,[], 1);
BzR = reshape(BZ,[], 1);
xR = reshape(MapX,[], 1);
yR = reshape(MapY,[], 1);
zR = reshape(MapZ,[], 1);
Bmap_raw = [BxR,ByR,BzR];
LocMap = [xR,yR,zR];

Bmap = Bmap_raw;
xMin = min(Bmap(:,1));
yMin = min(Bmap(:,2));
zMin = min(Bmap(:,3));
demX = max(Bmap(:,1))- xMin;
demY = max(Bmap(:,2))- yMin;
demZ = max(Bmap(:,3))- zMin;
Norm_para = [demX,demY,demZ,xMin,yMin,zMin];

for i = 1:size(Bmap,1)
    BxR(i) = (Bmap(i,1)-xMin)/demX;
    ByR(i) = (Bmap(i,2)-yMin)/demY;
    BzR(i) = (Bmap(i,3)-zMin)/demZ;
end

Xsize = size(BX,1);
Ysize = size(BX,2);
Zsize = size(BX,3);
BX_N = reshape(BxR,[Xsize,Ysize,Zsize]);
BY_N = reshape(ByR,[Xsize,Ysize,Zsize]);
BZ_N = reshape(BzR,[Xsize,Ysize,Zsize]);
magMap_norm = {BX_N,BY_N,BZ_N};
locMap = {MapX,MapY,MapZ};

%% initialize
mag_normlize = @(mag) [(mag(1)-Norm_para(4))/Norm_para(1),(mag(2)-Norm_para(5))/Norm_para(2),(mag(3)-Norm_para(6))/Norm_para(3)];
I = knnsearch([BxR,ByR,BzR], mag_normlize(magnetic(1,:)), 'K', 1);
location = LocMap(I,:); 
velocity = [0;0;0];

Euler = Euler_by_acc(acc(:,1))*pi/180;
quat = q_byEuler(Euler);
acc_correct(:,1) = Rot_by_Eulers(Euler(1), Euler(2), Euler(3))*acc(:,1);

cov_acc = sigmaX(1);
velocity_covQ = sigmaX(2);   % acc error cov
posit_covQ = sigmaX(3);

posit_covR = 1e-1*eye(1);   % mag error cov
velocity_covR = posit_covR*10;

covQ = [velocity_covQ, 0; 0, posit_covQ];
covR0 = [velocity_covR, 0; 0, posit_covR];  
covR = covR0;
covR_K = 1;

X_post(:,1) = [velocity(1,1); location(1,1)];
X_pri(:,1) = X_post(:,1);
P_post = zeros(2,2);

%% EKF
i = 2;
dim = 1;
acc_static_3std = 0.03;  % m/s2
acc_mean = 9.8;
gyro_static_3std = 0.05;  % rad

disp_cum = location(1,1);
net = resetState(net);
offset = 10;

while i <= size(acc,2)
    % heading direction
    acc_amp(i) = norm(acc(:,i));
    gyro_amp(i) = norm(gyro(:,i));
    mag_norm(i) = norm(magnetic(:,i));
    
    acc_diff_norm(i) = norm(acc(:,i) - acc(:,i-1));
    acc_is_static(i) = acc_amp(i) < acc_mean+acc_static_3std && acc_amp(i) > acc_mean-acc_static_3std ;
    gyro_is_static(i) = gyro_amp(i) < gyro_static_3std;
    
    Euler_acc(:,i) = Euler_by_acc(acc(:,i));
    quat(:,i) = quat_iteration(gyro(:,i),Ts)*quat(:,i-1);
    Euler_quat(:,i) = Euler_by_quat(quat(:,i)); 
    
    if gyro_is_static(i) == 0
        Euler(:,i) = Euler_quat(:,i);
    elseif acc_is_static(i) == 1
        Euler(:,i) = Euler_acc(:,i);
        quat(:,i) = q_byEuler(Euler(:,i));
    else
        Euler(:,i) = Euler(:,i-1);
    end
    
%     acc_correct(:,i) = Rot_by_Eulers(Euler(1,i), Euler(2,i), Euler(3,i))*acc(:,i)*100;
    acc_correct(:,i) = acc(:,i)*100;
    
    if acc_diff_norm(i) >= 0.2
        velocity(i) = X_post(1,i-1) + acc_correct(dim,i)*Ts;        
    else
        velocity(i) = 0;
    end
    
    disp_delta(i) = (velocity(i-1) + acc_correct(dim,i)*Ts/2)*Ts;
    XInputs = ([acc_correct(dim,i); velocity(i); disp_delta(i)] - muX) ./ sigmaX;
    
%     disp_cum(i) = X_post(2,i-1) + disp_delta(i);
%     X_pri(:,i) = [velocity(i); disp_cum(i)];
    
    if i <= offset
        [net,~] = predictAndUpdateState(net,XInputs);
        Y(i) = disp_delta(i);
    else
        [net,Y(i)] = predictAndUpdateState(net,XInputs);
    end
    disp_cum(i) = X_post(2,i-1) + denormalize(Y(i),sigmaT,muT);
    X_pri(:,i) = [velocity(i); disp_cum(i)];
    
    %% measurement by mag
    if mag_norm(i) > 0.1
        covR = covR0;
        mag_R = mag_normlize(magnetic(:,i));
        [location(i,:),I(i)] = knn_narrow(magMap_norm, locMap, I(i-1), 15, mag_R);
        velocity_mag(i) = (location(i,1) - location(i-1,1))/Ts;
    else
        covR = covR + (acc_amp(i)-9.8) * covR_K;
        location(i,:) = location(i-1,:);
        I(i) = I(i-1);
        velocity_mag(i) = velocity(i);
    end

    %% KF
    A = [1,0; Ts,1]; 
    P_pri = A*P_post*A' + covQ;
    
    H = [1,0; 0,1];
    Kk = P_pri*H'/(H*P_pri*H' + covR);

    innovation(:,i) = Kk*([velocity_mag(i); location(i,1)] - X_pri(:,i));
    X_post(:,i) = X_pri(:,i) + innovation(:,i);

    P_post = (eye(size(Kk,1))- Kk*H)*P_pri;     

    i = i+ 1;    
end  
    
%% plot data
figure
% subplot(121)
plot(location(:,1), 'LineWidth',2)
hold on
plot(X_post(2,:)', 'o')
hold on
plot(X_pri(2,:)','LineWidth',2)
legend('magnetic measurement','posterior','prediction')
xlabel('sample number')
ylabel('X location(cm)')

%% 
% load('DataSet_forTest_2.mat')
% acc_correct_ref = InputCell{1}(1,:);
% figure
% plot(acc_correct(1,:) - acc_correct_ref)
% plot(velocity' - InputCell{1}(2,:))
% plot(disp_delta - InputCell{1}(3,:))
