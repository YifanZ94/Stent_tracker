clear; clc
data = load('rotate_IMU.txt');
data = data(:,7:9);
% data = data';
xdata = data(:,1);
ydata = data(:,2);
zdata = data(:,3);

%%
myfun = @(x) (xdata-x(1)).^2 + (ydata-x(2)).^2 + (zdata-x(3)).^2 - x(4).^2;
p0 = [0,0,0,0];
lb = [-1, -1, -1, 0.01];
ub = [1, 1, 1, 1];
options = optimoptions(@lsqnonlin,'Algorithm','trust-region-reflective');
options.Algorithm = 'levenberg-marquardt';
x = lsqnonlin(myfun, p0, [], [], options);

%% plot
x_cali = xdata - x(1);
y_cali = ydata - x(2);
z_cali = zdata - x(3);
Y_cali = sqrt(x_cali.^2+y_cali.^2+z_cali.^2);
radius = mean(Y_cali);

[xR,yR,zR] = sphere;
surf(xR*radius, yR*radius, zR*radius, 'FaceAlpha',0.4,'EdgeColor','none') 
hold on
plot3(x_cali, y_cali, z_cali,'ro')
xlabel('X(cm)')
ylabel('Y(cm)')
zlabel('Z(cm)')
%%

