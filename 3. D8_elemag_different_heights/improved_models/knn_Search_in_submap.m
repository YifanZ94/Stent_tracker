clear; clc
file = {'H6.5_LIS.txt';
    'H8_LIS.txt';
    'H10_LIS.txt';
    'H11_LIS.txt';
    'H13_LIS.txt';
    'H15_LIS.txt';
    'H17_LIS.txt'};
H = {6.5,8,10,11,13,15,17};
Y_off = {-1,-1,-1,1,0,0,-1};
X_off = {1,-1,0,1,-1,0,0};
test_group = 1;
test = load(file{test_group});

%% square ref
load('Square_X40_Y20_Z15_reso0.2.mat')
C = 1.8;
% C = 1;
BxR = reshape(BX,[], 1); 
ByR = reshape(BY,[], 1);
BzR = reshape(BZ,[], 1);
xR = reshape(MapX,[], 1);
yR = reshape(MapY,[], 1);
zR = reshape(MapZ,[], 1);
Bmap_raw = [BxR,ByR,BzR];
LocMap = [xR,yR,zR];

%%
test = test/C;

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

%% first location
tic
measure_R = [(test(1,1)-para(4))/para(1), (test(1,2)-para(5))/para(2), (test(1,3)-para(6))/para(3)];
I(1) = knnsearch([BxR,ByR,BzR], measure_R, 'K', 1);
location = LocMap(I,:); 

%% submap searching
for j = 2:size(test,1)
    measure_R(j,:) = [(test(j,1)-para(4))/para(1), (test(j,2)-para(5))/para(2), (test(j,3)-para(6))/para(3)];
    [location(j,:),I(j)] = knn_narrow(magMap_norm, locMap, I(j-1), 15, measure_R(j,:));
end
toc

%%
figure
subplot(131)
xRef = (-15:1:15) + 0.5*X_off{test_group};
plot(xRef,location(:,1),xRef,xRef,'--')
xlabel('real location(cm)')
ylabel('predicted location(cm)')
title('X')
subplot(132)
yRef = zeros(31,1) + 0.5*Y_off{test_group};
plot(xRef,location(:,2))
xlabel('X reference(cm)')
ylabel('Y axis(cm)')
title('Y')
subplot(133)
zRef = -1*H{test_group}*ones(31,1);
plot(xRef,location(:,3));
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

