%% real data
clear; clc
load('B_10-30.mat','-mat');
load('normPara.mat','-mat');
mag = load('MMC_18_env.txt');
env = load('MMC_18_mag.txt');
test = mag - env;
% test = mag;

% load('mag_measurement_with_disturb.mat');
% test = test + data_env;

demX = p(1);
demY = p(2);
demZ = p(3);
xMin = p(4);
yMin = p(5);
zMin = p(6);

%%
Rx = @(roll) [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)];
Ry = @(pitch) [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)];
Rz = @(yaw)  [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1];
%% KNN
angle = 45*pi/180;
for j = 1:size(test,1)
    test_in_rotated_frame(j,:) = Rx(-angle)*test(j,:)';
end

measured_angle = 3*pi/180;
for j = 1:size(test,1)
    measure = Ry(measured_angle)*test_in_rotated_frame(j,:)';
    measure_R = [(measure(1)-xMin)/demX, (measure(2)-yMin)/demY, (measure(3)-zMin)/demZ];
    
    D = zeros(1,size(BxR2,1));
    for i = 1:size(BxR2,2)
        D(i) = (BxR2(i)- measure_R(1))^2 + (ByR2(i)- measure_R(2))^2 + (BzR2(i)- measure_R(3))^2;
    end
    [t, I(j)] = min(D);
    
    location(j,:) = [xR(I(j)) yR(I(j)) zR(I(j))];  
end
disp('done')

%% plot
location(:,1) = location(:,1) + 1;
location(:,2) = location(:,2) -1;
xRef = -15:0.5:15;
yRef = zeros(61,1);
zRef = -21*ones(61,1);
plot(xRef,location(:,1),xRef,xRef,'--')
legend('test','reference')
xlabel('real location(cm)')
ylabel('predicted location(cm)')
title('X')
figure

plot(xRef,location(:,2))
xlabel('X reference(cm)')
ylabel('Y axis(cm)')
title('Y')
figure

plot(xRef,location(:,3))
xlabel('X reference(cm)')
ylabel('Z axis(cm)')
title('Z')
%%
diff_vec = mean([abs(location(:,1)-xRef'), abs(location(:,2)-yRef), abs(location(:,3)-zRef)]);
