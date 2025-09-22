%% real data
clear; clc
load('Bmap_3D_raw_reso_0.2.mat');
load('LocMap_reso_0.2.mat');
env = load('MMC_18_env.txt');
mag = load('MMC_18_mag.txt');
test = env-mag;

%% model data
% load('Bmap_2D_reso_0.5.mat');
% height_dim = 11;
% theta = 0;
% Bx = Brho(height_dim,:)*cos(theta);
% By = Brho(height_dim,:)*sin(theta);
% Bz = Bz(height_dim,:);
% C = 1.2;
% test = C*[Bx', By', Bz'];
%%
C = 1.6;
Bmap = C*Bmap_raw;
xMin = min(Bmap(:,1));
yMin = min(Bmap(:,2));
zMin = min(Bmap(:,3));
demX = max(Bmap(:,1))- xMin;
demY = max(Bmap(:,2))- yMin;
demZ = max(Bmap(:,3)) - zMin;
para = [demX,demY,demZ,xMin,yMin,zMin];

for i = 1:size(Bmap,1)
    BxR(i) = (Bmap(i,1)-xMin)/demX;
    ByR(i) = (Bmap(i,2)-yMin)/demY;
    BzR(i) = (Bmap(i,3)-zMin)/demZ;
end

%% rotation matrix
theta = 0;

%% KNN
% size(test,1)
for j = 1:size(test,1)
    measure = [test(j,1)*cos(theta)-test(j,2)*sin(theta), test(j,1)*sin(theta)+test(j,2)*cos(theta), test(j,3)];
    measure_R(j,:) = [(measure(1)-para(4))/para(1), (measure(2)-para(5))/para(2), (measure(3)-para(6))/para(3)];
    
    I(j) = knnsearch([BxR',ByR',BzR'], measure_R(j,:), 'K', 1);
    location(j,:) = [LocMap(I(j),:)]; 
    matched_mag(j,:) = [BxR(I(j)),ByR(I(j)),BzR(I(j))];
end

%%
figure
subplot(131)
% xRef = -15:0.5:15;
xRef = linspace(-15,13.5,61);
plot(xRef,location(:,1),xRef,xRef,'--')
xlabel('real location(cm)')
ylabel('predicted location(cm)')
title('X')

subplot(132)
yRef = zeros(61,1);
plot(xRef,location(:,2))
xlabel('X reference(cm)')
ylabel('Y axis(cm)')
title('Y')

subplot(133)
zRef = -13*ones(61,1);
plot(xRef,location(:,3));
xlabel('X reference(cm)')
ylabel('Z axis(cm)')
title('Z')

% set(gca,'FontSize',15)
%% error
diff_vec = [(location(:,1)-xRef'), (location(:,2)-yRef), (location(:,3)-zRef)];
error_mean = [mean(abs((location(:,1)-xRef'))), mean(abs((location(:,2)-yRef))), mean(abs(location(:,3)-zRef))];
error_center = [location(31,1), location(31,2), location(31,3)+18];
error_std = [std(abs((location(:,1)-xRef'))), std(abs((location(:,2)-yRef))), std(abs(location(:,3)-zRef))];