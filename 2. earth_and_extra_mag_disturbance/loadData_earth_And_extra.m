clear; clc
data1 = load('H15_IMU_PerMag.txt');
data2 = load('env_IMU_PerMag.txt');
data3 = load('MMC_21.7_mag.txt');

% data1(1,:) = 0.95*data1(2,:);

data1 = data1 - data2;

mag1 = rms(data1,2);
mag2 = rms(data2,2);
mag3 = rms(data3,2);

%%
x = -15:0.5:15;
figure
subplot(221)
plot(x,mag1)
hold on
plot(x,mag2)
hold on
plot(x,mag3)
xlabel('X direction location (cm)')
ylabel('Flux Magnitude (Gauss)')
xlim([-15 15])
legend('Reference Mag','Background 1','Background 2')
% set(gca,'FontSize',15)

subplot(222)
plot(x,data1(:,1))
hold on
plot(x,data2(:,1))
hold on
plot(x,data3(:,1))
xlabel('X direction location (cm)')
ylabel('Flux in X (Gauss)')
xlim([-15 15])
% legend('Reference Mag','Background 1','Background 1')
% set(gca,'FontSize',15)

subplot(223)
plot(x,data1(:,2))
hold on
plot(x,data2(:,2))
hold on
plot(x,data3(:,2))
xlabel('X direction location (cm)')
ylabel('Flux in Y (Gauss)')
xlim([-15 15])
% legend('Reference Mag','Background 1','Background 1')
% set(gca,'FontSize',15)

subplot(224)
plot(x,data1(:,3))
hold on
plot(x,data2(:,3))
hold on
plot(x,data3(:,3))
xlabel('X direction location (cm)')
ylabel('Flux in Z (Gauss)')
xlim([-15 15])
% legend('Reference Mag','Background 1','Background 1')
% set(gca,'FontSize',15)
