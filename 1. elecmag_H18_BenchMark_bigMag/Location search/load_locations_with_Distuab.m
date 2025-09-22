clear; clc
load('MMC_18_locations.mat')
load('MMC_18_locations_with_distb.mat')

xRef = (-15:0.5:15);
yRef = zeros(61,1);
zRef = -18*ones(61,1);
figure
scatter(xRef,location(:,1),60,'ro')
hold on
scatter(xRef,location_D(:,1),'bx')
hold on
plot(xRef,xRef,'k--','LineWidth',2)
xlabel('X axis location(cm)')
ylabel('Measurement in X(cm)')
xlim([-15 15])
legend('corrected', 'without correction', 'reference')
set(gca,'FontSize',15)

figure
scatter(xRef,location(:,2),'ro')
hold on
scatter(xRef,location_D(:,2),'bx')
hold on
plot(xRef,yRef,'k--','LineWidth',2)
xlabel('X axis location(cm)')
ylabel('Measurement in Y(cm)')
ylim([-5 5])
legend('corrected', 'without correction', 'reference')
set(gca,'FontSize',15)

figure
scatter(xRef,location(:,3),'ro')
hold on
scatter(xRef,location_D(:,3),'bx')
hold on
plot(xRef,zRef,'k--','LineWidth',2);
ylim([-20 -15])
xlabel('X axis location(cm)')
ylabel('Measurement in Z(cm)')
legend('corrected', 'without correction', 'reference')
set(gca,'FontSize',15)

%%
error = [xRef'-location(:,1), yRef-location(:,2), zRef-location(:,3)];
error_ave = mean(abs(error));
error_std = std(abs(error));