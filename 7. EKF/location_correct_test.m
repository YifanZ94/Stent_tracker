clear; clc
data = load('Test1_state.txt');

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

magnetic = data(:,7:9);
magnetic(:,3) = -magnetic(:,3);
magnetic_ori = magnetic;
magnet_on = data(:,10);
Ts = 0.0145;
L = size(magnetic,1);

%% extra mag disturbance
distub_period = 8;
B_distb = 0.4;

extra_mag_distb_X = B_distb*sawtooth(2*pi/distub_period*Ts*(1:L), 0.5)';
% 
% % extra_mag_distb_X = B_distb*sin(2*pi/distub_period*Ts*(1:L))';
% 
magnetic(:,1) = magnetic(:,1) + extra_mag_distb_X;
magnetic(:,2) = magnetic(:,2) + extra_mag_distb_X;
magnetic(:,3) = magnetic(:,3) + extra_mag_distb_X;

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

%%
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
f=fit((1:size(location_mag_off,1))', location_mag_off(:,1)-location_mag_off(1,1), 'poly1');
location_correct = location;
last_mag_on_idx = mag_start_ind;

for i = 2:L-mag_start_ind
    
    j = i + mag_start_ind;
    if magnet_on(j) == 0
        mag_distb = magnetic(j,:);
    else
        mag_ref = magnetic(j,:);
    end

    mag_corrected(j,:) = mag_ref - mag_distb;
    mag_R = mag_normlize(mag_corrected(j,:));
    if  magnet_on(j) ~=  magnet_on(j-1)
        I(i) = knnsearch([BxR,ByR,BzR], mag_R, 'K', 1);
        location(i,:) = LocMap(I(i),:);  
    else
        [location(i,:), I(i)] = knn_narrow(magMap_norm, locMap, I(i-1), 15, mag_R);
    end

    if magnet_on(j) == 0
        location_mag_off = [location_mag_off; location(i,:)];
        location_correct(i,:) = location(i,:);
        mag_error(i) = NaN;
    elseif magnet_on(j-1) == 0
        last_mag_on_idx = j;
        f=fit((1:size(location_mag_off,1))', location_mag_off(:,1)-location_mag_off(1,1), 'poly1');
        location_correct(i,:) = location(i,:);
        location_mag_off = []; 
        mag_error(i) = NaN;
    else
        temp_idx = (j-last_mag_on_idx);   
%         mag_error(i) = f.p1*temp_idx^2 + f.p2*temp_idx + f.p3;      
        mag_error(i) = f.p1*temp_idx;      
        location_correct(i,:) = location(i,:) - mag_error(i);
    end
end
    
%% 
idx_mag_off = find(magnet_on(mag_start_ind:end)==0);
idx_mag_on = find(magnet_on(mag_start_ind:end)==1);
location_mag_off = location;
idx_mag_on(1) = 2;
location_mag_off(idx_mag_on-1,:) = NaN;
location_mag_on = location;
location_mag_on(idx_mag_off,:) = NaN;

figure
plot(location_mag_off(:,1), 'x')
% hold on
% plot(location_mag_on(:,1))
% hold on
% plot(location_correct(:,1))
% hold on
% plot(mag_error)
hold on
plot(location(:,1))
% hold on
% plot(location_correct(:,1))
% legend('original','corrected')
