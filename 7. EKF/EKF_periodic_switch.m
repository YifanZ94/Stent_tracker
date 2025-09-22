clear; clc
data = load('Test3_state.txt');

%% Add a delay after the switch
periods = floor(size(data,2)/120);
mag_state_idx = 0;
magnet_on = ones(1,size(data,2));
for i = 1:periods
    mag_state_idx = [mag_state_idx, i*20, i*120];
    magnet_on((i-1)*120+2 : (i-1)*120+22 ) = 0;
end
mag_state_idx = mag_state_idx + 1;

% skip_idx = [mag_state_idx,mag_state_idx+1,mag_state_idx+2];
% data(:,skip_idx) = [];
% magnet_on(skip_idx) = [];

idx_mag_switch = find( (magnet_on(1:end-1)-magnet_on(2:end)) ~= 0)+3; 

%% initial stage trim
init_range = 1;
acc_init = data(1:3,1);
gyro_init = data(4:6,1);

acc = data(1:3,:);
gyro = (data(4:6, :) - gyro_init)*pi/180;
magnetic = data(7:9, :)';

% magnetic(:,3) = -magnetic(:,3);
mag_start_ind = 1;
mag_ref = magnetic(1,:);
mag_distb = magnetic(2,:);
mag_corrected = mag_ref;


Ts = 0.0145;
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

Euler = Euler_by_acc(acc_init);
acc_correct(:,1) = Rot_by_Eulers(Euler(1), Euler(2), Euler(3))*acc_init;
quat(:,1) = q_byEuler(Euler);
% Euler_quat = Euler_by_quat(quat)*180/pi; 
Euler_quat = Euler;

%% static thresholds
for i = 1:L
%     acc_amp(i) = norm(acc(:,i));
    acc_amp(i) = abs(norm(acc(:,i)) - norm(acc_init));
    gyro_amp(i) = norm(gyro(:,i));
%     mag_amp(i) = norm(magnetic(i,:));
end

a0 = 0.02;
w0 = 0.02;
% a1 = 0.12;
% w1 = 0.12;
a1 = 0.05;
w1 = 0.05;

move_window = 20;
acc_amp_ave = movmean(acc_amp, move_window);
gyro_amp_ave = movmean(gyro_amp, move_window);

% acc_amp_ave = abs(acc_amp_ave - norm(acc_init));
acc_amp_ave = abs(acc_amp_ave - min(acc_amp_ave));
gyro_amp_ave = gyro_amp_ave - min(gyro_amp_ave);

for i = 1:L
    acc_abs_static(i) = acc_amp_ave(i) < a0;
    gyro_abs_static(i) = gyro_amp(i) < w0;

%     acc_is_static(i) = acc_diff_norm(i) < a_diff_0;
%     gyro_is_static(i) = gyro_amp(i) < a_diff_0;
end

for i = 2:L
    Euler_acc(:,i) = Euler_by_acc(acc(:,i));
    quat(:,i) = quat_iteration(gyro(:,i),Ts)*quat(:,i-1);
    Euler_quat(:,i) = Euler_by_quat(quat(:,i)); 
    
    if gyro_abs_static(i) == 0
        Euler(:,i) = Euler_quat(:,i);
        elseif acc_abs_static(i) == 1
            Euler(:,i) = Euler_acc(:,i);
            quat(:,i) = q_byEuler(Euler(:,i));
        else
            Euler(:,i) = Euler(:,i-1);
    end
    acc_correct(:,i) = Rot_by_Eulers(Euler(1,i), Euler(2,i), Euler(3,i))*acc(:,i);
    
end

% a_std_0 = 0.01;
a_std_0 = 0.02;
w_std_0 = 0.01;
H = 5;

for i = H:L
    acc_amp_std(i-H+1) = std(acc_correct(1, i-H+1:i));
end
acc_amp_std = [zeros(1,H-1), acc_amp_std];
acc_amp_std = movmean(acc_amp_std, 5);

velocity = 0;
disp_delta = 0;

for i = 2:L
    acc_std_is_static(i) = acc_amp_std(i) < a_std_0;
    if acc_std_is_static(i) == 0
        velocity(:,i) = velocity(:,i-1) + acc_correct(dim,i)*Ts;
    else
        velocity(:,i) = 0;
    end
    disp_delta(:,i) = (velocity(:,i) + velocity(:,i-1))*Ts/2;
    
end

state_moveM = movmean(acc_std_is_static, 10);
for i = 1:L
    if state_moveM(i) > 0.5
        state_actvitate(i) = 1;
    else
        state_actvitate(i) = 0;
    end
end

covQ = 0.1;   
covR = 0.1;   

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
    NN_inputs = [acc_correct(dim,i); velocity(:,i); disp_delta(:,i); acc_std_is_static(i); gyro_amp(i)]./sigmaX;
%     NN_inputs = [acc_correct(dim,i); velocity(:,i); disp_delta(:,i); acc_std_static(i); gyro_amp(i)];
    [net,Y(:,i)] = predictAndUpdateState(net,NN_inputs);
   
    if acc_std_is_static(i) == 1
        Y(:,i) = 0;
%         P_pri(i) = 0;
    else
%         P_pri(i) = P_post(i-1) + 0.1;
%         P_pri(i) = 0.1;
        Y(:,i) = max(Y(:,i)*sigmaT*100, 0);
    end
    disp_sum(:,i) = disp_sum(:,i-1) + Y(:,i);
%     X_pri(:,i) = X_post(:,i-1) + Y(:,i);

    mag_R = mag_normlize(magnetic(i,:));
    
    if  magnet_on(i) ~=  magnet_on(i-1)
        I(i) = knnsearch([BxR,ByR,BzR], mag_R, 'K', 1);
        location(i,:) = LocMap(I(i),:);  
        Z(i) = location(i,1);
    else
        [location(i,:), I(i)] = knn_narrow(magMap_norm, locMap, I(i-1), 15, mag_R);
        if acc_std_is_static(i) == 1 
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
        
        moving_idx = find(acc_std_is_static(last_range) == 0);
        disp_ref_slope = (disp_end - disp_start)/size(moving_idx,2);
        
        for j = last_range(2:end)
            if acc_std_is_static(j) == 1 
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

%%
% Xpost_pi = load('static_dist_slow_xPost.txt');
% figure
% plot(Xpost_pi)
% hold on
% plot(X_post)
% xlabel('sample')
% ylabel('Error (cm)')
% legend('mag', 'inertial')

%% plot locations
figure
scatter(idx_mag_switch, location(idx_mag_switch,1))
hold on
% scatter(idx_mag_switch, Z(idx_mag_switch))
% hold on
plot(X_post, 'LineWidth', 2)

% save('Test1_KF2_distb0.2.mat', 'X_post', 'location', 'disp_sum', 'mag_start_ind', 'mag_switch_idx')
