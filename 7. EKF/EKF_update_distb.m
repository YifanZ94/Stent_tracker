clear; clc
data = load('Test4_state.txt');

%% Add a delay after the switch
magnet_on = data(:,10);
mag_switch_idx = [];
for i = 2:size(data,1)
    if magnet_on(i) ~= magnet_on(i-1)
        mag_switch_idx = [mag_switch_idx,i];
    end
end
skip_idx = [mag_switch_idx,mag_switch_idx+1,mag_switch_idx+2];
data(skip_idx,:) = [];

%%
init_range = 10;
acc_init = mean(data(1:init_range, 1:3), 1);
gyro_init = mean(data(1:init_range, 4:6), 1);

magnetic = data(:,7:9);
magnetic(:,3) = -magnetic(:,3);
magnetic_ori = magnetic;
magnet_on = data(:,10);
Ts = 0.0145;
L = size(magnetic,1);

%% high frequency distub
% B_distb = 0.3;
% extra_mag_distb_X = B_distb*rand(L,1) - B_distb/2;
% X = magnetic + extra_mag_distb_X;
% magnetic(:,2) = magnetic(:,2) + extra_mag_distb_X;
% magnetic(:,3) = magnetic(:,3) + extra_mag_distb_X;

%% low frequency extra mag disturbance
distub_period = 6;
B_distb = 0.3;
extra_mag_distb_X = B_distb*sawtooth(2*pi/distub_period*Ts*(1:L), 0.5)';
% % extra_mag_distb_X = B_distb*sin(2*pi/distub_period*Ts*(1:L))';
% 
% magnetic(:,1) = magnetic(:,1) + extra_mag_distb_X;
X = magnetic + extra_mag_distb_X;

%%
% Fs = 1/Ts;
% % X = magnetic(:,1);
% Y = fft(X);
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
% f_cut = 0.001;
% data_LP = lowpass(X,f_cut,Fs,Steepness=0.95);
% figure
% plot(magnetic(:,1))
% hold on
% plot(data_LP(:,1))
% xlabel('sampling number')
% ylabel('acc in X (cm2/s)')
% legend('raw data','low pass')
% 
% magnetic = data_LP;

%% magnetic correction
% mag_start_ind = find(magnet_on==1,1);
magnetic = X;
idx_mag_on = find(magnet_on == 1); 
idx_mag_off = find(magnet_on == 0); 
idx_mag_switch = find( (magnet_on(1:end-1)-magnet_on(2:end)) ~= 0)+1; 

%%
acc = data(:, 1:3);
gyro = (data(:, 4:6) - gyro_init)*pi/180;
acc = acc';
gyro = gyro';

velocityX_raw = cumtrapz(Ts, acc(:,1));
locationX_raw = cumtrapz(Ts, velocityX_raw);

dim = 1;

%% extra vibration
% L = size(acc,2);
% vib_std = 0.3;
% vib_X = vib_std*rand(1,L) - vib_std/2;
% 
% acc(1,:) = acc(1,:) + vib_X;
% acc(2,:) = acc(2,:) + vib_X;
% acc(3,:) = acc(3,:) + vib_X;
% plot(acc(:,1))

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

mag_start_ind = find(magnet_on==1, 1);
mag_ref = magnetic(mag_start_ind,:);
mag_distb = magnetic(mag_start_ind-1,:);

mag_corrected = mag_ref-mag_distb;

I = knnsearch([BxR,ByR,BzR], mag_normlize(mag_corrected), 'K', 1);
location = LocMap(I,:); 

I_init = I;
location_mag_off = location;
for i = 2:mag_start_ind-1
    mag_R = mag_corrected + magnetic(mag_start_ind-i,:);
    [location_mag_off(i,:), I_init(i)] = knn_narrow(magMap_norm, locMap, I_init(i-1), 15, mag_R);
    location_mag_off(i,:) = LocMap(I,:);
end

% f=fit((1:size(location_mag_off,1))', location_mag_off(:,1)-location_mag_off(1,1), 'poly2');
location_init = location_mag_off;
covR_slope = (max(location_init,[],1) - min(location_init,[],1))/mag_start_ind ;
covR_slope = covR_slope(1);

Euler = Euler_by_acc(acc_init);
acc_correct(:,1) = Rot_by_Eulers(Euler(1), Euler(2), Euler(3))*acc_init';
quat(:,1) = q_byEuler(Euler);
% Euler_quat = Euler_by_quat(quat)*180/pi; 
Euler_quat = Euler;

%% static thresholds
L = size(acc,2);
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

move_window = 10;
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
        else
            Euler(:,i) = Euler(:,i-1);
    end
    acc_correct(:,i) = Rot_by_Eulers(Euler(1,i), Euler(2,i), Euler(3,i))*acc(:,i);
    
end

% a_std_0 = 0.01;
a_std_0 = 0.06;
w_std_0 = 0.01;
H = 5;
for i = H:L
    acc_amp_std(i-H+1) = std(acc_correct(1, i-H+1:i));
end
acc_amp_std = [zeros(1,H-1), acc_amp_std];

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

covQ = 0.1;   
covR = 0.1;   

X_post(:,1) = location(1);
X_pri(:,1) = X_post(:,1);
P_post = covQ;
% P_post = zeros(1,1);

%% NN load
load('Trained_NN_both_standard.mat')
net = resetState(net);
sigmaX(4) = 1;
% offset = 10;
% [net,~] = predictAndUpdateState(net,XTest(:,1:offset));

%% EKF
% load('location_distb_0.3.mat')
disp_sum = X_post(:,1);
Z = X_post(:,1);
H = 1;
varR = 0.1*ones(1,3);
last_mag_on_idx = mag_start_ind;

for i = 2:L-mag_start_ind
    j = i + mag_start_ind;
    
    if magnet_on(j) == 0
        mag_distb = magnetic(j,:);
    else
        mag_ref = magnetic(j,:);
    end
    
    % NN implementation
    NN_inputs = [acc_correct(dim,j); velocity(:,j); disp_delta(:,j); acc_std_is_static(j); gyro_amp(j)]./sigmaX;
%     NN_inputs = [acc_correct(dim,i); velocity(:,i); disp_delta(:,i); acc_std_static(i); gyro_amp(i)];
    [net,Y(:,i)] = predictAndUpdateState(net,NN_inputs);
   
    if acc_std_is_static(j) == 1
        Y(:,i) = 0;
        P_pri(i) = 0;
        if magnet_on(j) == 1 && magnet_on(j-1) == 1
            mag_distb = mag_distb + magnetic(j,:) - magnetic(j-1,:);
        end
    else
%         P_pri(i) = P_post(i-1) + 0.1;
        P_pri(i) = 0.1;
        Y(:,i) = Y(:,i)*sigmaT*100;
    end
    disp_sum(:,i) = disp_sum(:,i-1) + Y(:,i);
    X_pri(:,i) = X_post(:,i-1) + Y(:,i);
    
    mag_corrected(j,:) = mag_ref - mag_distb;
%     A = [1,0; Ts,1]; 
%     P_pri = A*P_post*A' + covQ;

    % magnetic measurements 
    mag_R = mag_normlize(mag_corrected(j,:));
    if  magnet_on(j) ~=  magnet_on(j-1)
        I(i) = knnsearch([BxR,ByR,BzR], mag_R, 'K', 1);
        location(i,:) = LocMap(I(i),:);  
        Z(i) = location(i,1);
    else
        [location(i,:), I(i)] = knn_narrow(magMap_norm, locMap, I(i-1), 15, mag_R);
        if acc_std_is_static(j) == 1 
            Z(i) = Z(i-1);
        else
            Z(i) = location(i,1);
        end
    end
   
    if magnet_on(j) == 1
        if magnet_on(j-1) == 0
            last_mag_on_idx = j;
%             f=fit((1:size(location_mag_off,1))', location_mag_off(:,1)-location_mag_off(1,1), 'poly1');

            covR_slope = varR(dim)/idx_mag_switch(1);
            location_mag_off = [];      
            covR(i) = 0;   
        else
%             temp_idx = j-last_mag_on_idx;   
%             mag_error = f.p1*temp_idx^2 + f.p2*temp_idx + f.p3;        
            covR(i) = varR(dim);
        end       
    else
        location_mag_off = [location_mag_off; location(i,:)];
%         varR = std(location_mag_off, [], 1);
        varR = (max(location_mag_off,[],1) - min(location_mag_off,[],1));
        covR(i) = varR(dim);  
    end
    
%     if magnet_on(j) == 1 && magnet_on(j-1) ~= 0
%         location_correct(i) = location(i) - mag_error;
%     else
%         location_correct(i) = location(i);
%     end

%    if magnet_on(j) ~=  magnet_on(j-1)
%         covR(i) = 0;    
%     end
    
    Kk(i) = min(P_pri(i)/(P_pri(i) + covR(i)), 1);
    X_post(:,i) = (1-Kk(i))*X_pri(:,i) + Kk(i)*Z(i);

    P_post(i) = (eye(size(Kk,1))- Kk(i))*P_pri(i);        
        
end  


%% plot data
% times = Ts*(1:size(location,1));
times = 1:size(location,1);
figure
plot(times, X_post, 'LineWidth', 2)
hold on
plot(times, location(:,1),'o')
hold on
plot(times, disp_sum,'x')
% hold on
% plot(times, X_pri,'x')

% legend('fusion','magnetic','inertial')
xlabel('sample number')
ylabel('X location(cm)')

%%
figure
plot(times, covR, 'x')
title('mag')
xlabel('sample')
ylabel('Error (cm)')
figure
plot(times, P_pri, 'o')
title('IMU')
xlabel('sample')
ylabel('Error (cm)')
% legend('mag', 'inertial')

%%
figure
scatter(idx_mag_switch(2:end)-mag_start_ind, location(idx_mag_switch(2:end)-mag_start_ind,1))
hold on
plot(times, X_post, 'LineWidth', 2)

% plot(magnetic_ori(:,1) )
% hold on
% % plot(magnetic(:,1)- extra_mag_distb_X)
% plot(mag_corrected(:,1))
% hold on
% plot(idx_mag_switch, magnetic_ori(idx_mag_switch,1),'x')
% legend('raw','corrected')

%%
save('Test1_KF_distb0.2.mat', 'X_post','location','X_pri', 'disp_sum')

%%
% figure
% plot(location(:,1))
% hold on
% plot(location_correct)
