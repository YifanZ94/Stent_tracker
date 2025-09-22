clear; clc
data = load('mag_below2_IMU_state.txt');
init_range = 10;
acc_init = mean(data(1:init_range, 1:3), 1);
gyro_init = mean(data(1:init_range, 4:6), 1);

%  - acc_init'
acc = data(init_range+1:end, 1:3)';
gyro = (data(init_range+1:end, 4:6) - gyro_init)'*pi/180;

magnetic = data(init_range+1:end,7:9);
magnetic(:,3) = -magnetic(:,3);
magnet_on = data(init_range+1:end,10);

%% square ref
load('Square_X40_Y20_Z15_reso0.2_T.mat')
C = 1;
BxR = reshape(BX,[], 1); 
ByR = reshape(BY,[], 1);
BzR = reshape(BZ,[], 1);
xR = reshape(MapX,[], 1);
yR = reshape(MapY,[], 1);
zR = reshape(MapZ,[], 1);
Bmap_raw = [BxR,ByR,BzR];
LocMap = [xR,yR,zR];

%%
test = magnetic/C;

Bmap = Bmap_raw;
xMin = min(Bmap(:,1));
yMin = min(Bmap(:,2));
zMin = min(Bmap(:,3));
demX = max(Bmap(:,1))- xMin;
demY = max(Bmap(:,2))- yMin;
demZ = max(Bmap(:,3))- zMin;
para = [demX,demY,demZ,xMin,yMin,zMin];

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

%% searching
tic
% 1:size(test,1)
for j = 143:153
    measure_R = [(test(j,1)-para(4))/para(1), (test(j,2)-para(5))/para(2), (test(j,3)-para(6))/para(3)];
    I(j) = knnsearch([BxR,ByR,BzR], measure_R, 'K', 1);
    location(j,:) = LocMap(I(j),:); 
end
toc

%%
figure
subplot(131)
plot(location(:,1))
xlabel('real location(cm)')
ylabel('predicted location(cm)')
title('X')
subplot(132)
plot(location(:,2))
xlabel('X reference(cm)')
ylabel('Y axis(cm)')
title('Y')
subplot(133)
plot(location(:,3));
xlabel('X reference(cm)')
ylabel('Z axis(cm)')
title('Z')

%% error
diff_vec = [(location(:,1)-xRef'), (location(:,2)-yRef), (location(:,3)+H{test_group})];
error_mean = [mean(abs((location(:,1)-xRef'))), mean(abs((location(:,2)-yRef))), mean(abs(location(:,3)-zRef))];
error_center = [diff_vec(16,1), diff_vec(16,2), diff_vec(16,3)];
error_std = [std(abs((location(:,1)-xRef'))), std(abs((location(:,2)-yRef))), std(abs(location(:,3)-zRef))];

%%
figure
subplot(131)
plot(xRef,diff_vec(:,1))
xlabel('X location(cm)')
ylabel('X error(cm)')
subplot(132)
plot(xRef,diff_vec(:,2))
xlabel('X location(cm)')
ylabel('Y error(cm)')
subplot(133)
plot(xRef,diff_vec(:,3))
xlabel('X location(cm)')
ylabel('Z error(cm)')


%% Location change with time
T1 = 0:9;
locX1 = repelem(location(1:5,1),2);
figure
plot(T1,locX1,'x--','MarkerSize', 12)
xlabel('Time (s)')
ylabel('Location  measurement in X (cm)')

%% Background mag change with time
mag_env = load('env_IMU_PerMag.txt');
envX1 = repelem(mag_env(1:5,1),2);
plot(T1, envX1, 'linewidth', 2)

hold on
for i = 1:2:7
    plot([i-1,i+1], [envX1(i+1),envX1(i+1)], 'o- r')
    hold on
end
legend('actual','measurement')

% t_rand = linspace(3,4);
% env_rand = envX1(4)+ 0.2*envX1(4)*sin(2*pi*t_rand);
% plot(t_rand,env_rand)

xlabel('Time (s)')
ylabel('Background Magnetic Strength (G)')

