clear; clc
data = load('t2_IMU.txt');
Euler = load('t2_Euler.txt');

% Euler = Euler(:, start_idx*2-1:2:end);   % for forward_rotate only

%% first identify the initial index
magnet_on = data(10,:);
magnet_on(1) = 1;
idx_mag_switch = find( (magnet_on(1:end-1)-magnet_on(2:end)) ~= 0); 
plot(Euler')

%% trim the data
start_idx = 165;
data = data(:, start_idx:end);
Euler = Euler(:, start_idx:end);

%%
magnet_on = data(10,:);
magnet_on(1) = 1;
idx_mag_switch = find( (magnet_on(1:end-1)-magnet_on(2:end)) ~= 0); 

%%
times = 0.01*(1:size(Euler,2));
plot(times, Euler')
xlabel('Time (s)')
ylabel('Euler angles (rads)')
legend('roll','pitch','yaw')
set(gca, 'Fontsize', 12)

%% initial stage trim
init_range = 1;
acc_init = data(1:3,1);
gyro_init = data(4:6,1);
move_state = data(11,:);

acc = data(1:3,:);
gyro = (data(4:6, :) - gyro_init)*pi/180;
magnetic = data(7:9, :)';
magnetic(:,3) = -magnetic(:,3);

mag_start_ind = 1;
mag_ref = magnetic(1,:);
mag_distb = magnetic(2,:);
mag_corrected = mag_ref;

Ts = 0.02;
L = size(magnetic,1);
dim = 1;

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
%                                 [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1];
                            
Rot_by_Eulers = @(roll,pitch,yaw) [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1]* ...
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
I = knnsearch([BxR,ByR,BzR], mag_normlize(mag_corrected), 'K', 1);
location = LocMap(I,:); 

%% velocity filtering
velocity = 0;
disp_delta = 0;
acc_correct = Rot_by_Eulers(Euler(1,1),Euler(2,1),Euler(3,1))*acc(:,1);

for i = 2:L
    gyro_amp(i) = norm(gyro(:,i));
    acc_correct(:,i) = Rot_by_Eulers(Euler(1,1),Euler(2,1),Euler(3,1))*acc(:,i);
    if move_state(i) == 1
        velocity(:,i) = velocity(:,i-1) + acc_correct(dim,i)*Ts;
    else
        velocity(:,i) = 0;
    end
    disp_delta(:,i) = (velocity(:,i) + velocity(:,i-1))*Ts/2;
end

disp_delta = -disp_delta;
X_post(:,1) = location(1);
X_pri(:,1) = X_post(:,1);

%% NN load
load('Trained_NN_both_standard.mat')
net = resetState(net);
sigmaX(4) = 1;
% offset = 10;
% [net,~] = predictAndUpdateState(net,XTest(:,1:offset));

%% EKF
% load('location_distb_0.3.mat')
disp_sum = X_post(:,1);
disp_ref = disp_sum;
Z = X_post(:,1);
H = 1;
Kk = 0.5;
Kk_next = 0.5;
varR = 0.1*ones(1,3);
last_switch_idx = 1;

for i = 2:L
    % NN implementation
    NN_inputs = [acc_correct(dim,i); velocity(:,i); disp_delta(:,i); move_state(i); gyro_amp(i)]./sigmaX;
%     NN_inputs = [acc_correct(dim,i); velocity(:,i); disp_delta(:,i); acc_std_static(i); gyro_amp(i)];
    [net,Y(:,i)] = predictAndUpdateState(net,NN_inputs);
   
    if move_state(i) == 1
        Y(:,i) = 0;
    else
        Y(:,i) = max(Y(:,i)*sigmaT*100, 0);
    end

%     Y(:,i) = max(Y(:,i)*sigmaT*100, 0);
    
    disp_sum(:,i) = disp_sum(:,i-1) + Y(:,i);

    mag_R = mag_normlize(magnetic(i,:));
    
    if  magnet_on(i) ~=  magnet_on(i-1)
        I(i) = knnsearch([BxR,ByR,BzR], mag_R, 'K', 1);
        location(i,:) = LocMap(I(i),:);  
        Z(i) = location(i,1);
    else
        [location(i,:), I(i)] = knn_narrow(magMap_norm, locMap, I(i-1), 15, mag_R);
        if move_state(i) == 1 
            Z(i) = Z(i-1);
        else
            Z(i) = location(i,1,1);
        end
        
        if Z(i)<Z(i-1)
            Z(i) = Z(i-1);
        end
    end
    
    if magnet_on(i) ~= magnet_on(i-1) && i > 4
        last_range = last_switch_idx: (i-1);
        Kk(i) = 1;
        
        disp_start = location(last_switch_idx,dim);
        disp_end = location(i,dim);
        disp_sum(i) = disp_end;
        disp_ref(last_range(1)) = disp_start;
        
        moving_idx = find(move_state(last_range) == 0);
        disp_ref_slope = (disp_end - disp_start)/size(moving_idx,2);
        
        for j = last_range(2:end)
            if move_state(j) == 1 
                disp_ref(j) = disp_ref(j-1);
            else
                disp_ref(j) = disp_ref(j-1) + disp_ref_slope;
            end
        end
        
        if magnet_on(i) == 1
            disp_mag_ref(last_range) = disp_start*ones(1,size(last_range,2));
        else
            disp_mag_ref(last_range) = disp_ref(last_range);
        end
        
        RMSE_mag(last_range) = mean((disp_mag_ref(last_range)-Z(last_range)).^2);
        RMSE_inertial(last_range) = mean((disp_ref(last_range)-disp_sum(last_range)).^2) +0.01;
        
        Kk_next =  min(RMSE_inertial(j)/(RMSE_inertial(j)+RMSE_mag(j)) , 1);
        last_switch_idx = i;
        
    else
        Kk(i) = Kk_next;
    end   
    
    X_post(:,i) = (1-Kk(i))*disp_sum(:,i) + Kk(i)*Z(i);
         
end  

%% plot data
times = Ts*(1:size(location,1));
% times = 1:size(location,1);
figure
plot(times, X_post, 'LineWidth', 2)
hold on
plot(times(1:10:end), location(1:10:end,1), 'o')
hold on
plot(times(1:10:end), disp_sum(1:10:end),'x')
% hold on
% plot(times, X_pri,'x')

legend('fusion','magnetic','inertial')
xlabel('Time (s)')
% xlabel('sample number')
ylabel('X location(cm)')

%% compare MAT with NN and Rasp Pi outouts
Xpost_pi = load('t2_x_post.txt');
figure
plot(Xpost_pi(1,start_idx:end))
hold on
plot(X_post)
xlabel('sample')
ylabel('Error (cm)')
legend('No NN', 'With NN')

% figure
% disp_raw = cumsum(disp_delta*100);
% disp_NN = cumsum(Y);
% plot(disp_raw)
% hold on
% plot(disp_NN)

%% plot locations
figure
scatter(idx_mag_switch, location(idx_mag_switch,1))
hold on
% scatter(idx_mag_switch, Z(idx_mag_switch))
% hold on
plot(X_post, 'LineWidth', 2)

% save('Test1_KF2_distb0.2.mat', 'X_post', 'location', 'disp_sum', 'mag_start_ind', 'mag_switch_idx')

%% plot Z measurement
Z_loc = 0.6*location(:,3)-3.6;
RMSE_Z = mean(abs(Z_loc - (-9*ones(size(location,1),1))));
plot(times, -9*ones(size(location,1),1), '--')
hold on
plot(times, Z_loc)
xlabel('Time (s)')
ylabel('Z location(cm)')
legend('Reference','Measurement')
set(gca, 'FontSize', 12)
%% add delay as the mag switch
idx_mag_range = [1, idx_mag_switch, size(X_post,2)];
sync_data = [];
output = [Xpost_pi(:,start_idx:end); magnet_on];
row_Length = size(output,1);

for i = 1:size(idx_mag_range,2)-1
    data_p = output(:, idx_mag_range(i): idx_mag_range(i+1)-1);
    if idx_mag_range(i+1) == 0
        pad_L = 5;
    else
        pad_L = 5;
    end
    sync_data = [sync_data, data_p, data_p(:,end).*ones(row_Length, pad_L)];
end

%%
% X_post_padd = sync_data(1,:);
% Y_post_padd = sync_data(2,:);
% Z_post_padd = sync_data(3,:);
% magnet_on_padd = sync_data(4,:);
% idx_mag_switch_padd = find( (magnet_on_padd(1:end-1)-magnet_on_padd(2:end)) ~= 0); 
% save('padded_XpostPi', 'X_post_padd', 'Y_post_padd', 'Z_post_padd', 'idx_mag_switch_padd')