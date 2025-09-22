clear; clc
load('H25_location.txt');
test1 = H25_location;

%%  common plot commands
xref = -15:0.5:15;
plot(xref,xref,'--',xref,test1(:,1))
xlabel('real location(cm)')
ylabel('measured location(cm)')
legend('reference','corrected','raw measurement')

figure
yref = zeros(1,61);
plot(xref,yref,'--',xref,test1(:,2))
xlabel('real location(cm)')
ylabel('measured location(cm)')
legend('reference','corrected','raw measurement')

figure
zref = -18*ones(1,61);
plot(xref,zref,'--',xref,test1(:,3))
xlabel('real location(cm)')
ylabel('measured location(cm)')
legend('reference','corrected','raw measurement')
