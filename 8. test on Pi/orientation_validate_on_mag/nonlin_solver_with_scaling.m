clear; clc
% data = load('tes_mag.txt');
% xdata = data(:,1);
% ydata = data(:,2);
% zdata = data(:,3);

%%  test example
% d = linspace(0,3);
% y = exp(-1.3*d) + 0.05*randn(size(d));
% fun = @(r)exp(-d*r)-y;

%% create sphere data
x = randn(100,1);
y = randn(100,1);
z = randn(100,1);
bias = [1,2,3];
for i = 1:100
    N = norm([x(i),y(i),z(i)]);
    xN(i) = x(i)/N + bias(1) + rand-0.5;
    yN(i) = y(i)/N + bias(2) + rand-0.5;
    zN(i) = z(i)/N + bias(3) + rand-0.5;
end
xdata = xN';
ydata = yN';
zdata = zN';

%%
myfun = @(x) (xdata-x(1)).^2 + (ydata-x(2)).^2 + (zdata-x(3)).^2 - x(4).^2;
p0_fun1 = [0,0,0,1];
p_1 = lsqnonlin(myfun,p0_fun1);

%%
myfun2 = @(x) (x(4)*xdata-x(1)).^2 + (x(5)*ydata-x(2)).^2 + (x(6)*zdata-x(3)).^2 + p_1(4)^2;
p0_fun2 = [1;1;1;1;1;1;];
scaling_lb = 0.8;
scaling_ub = 1.2;
lb = [-5*ones(3,1); scaling_lb*ones(3,1)];
ub = [5*ones(3,1); scaling_ub*ones(3,1)];

[p_2,resnorm] = lsqnonlin(myfun,p0_fun2,lb,ub);

%% plot
x_cali = xdata - p_2(1);
y_cali = ydata - p_2(2);
z_cali = zdata - p_2(3);
Y_cali = sqrt(x_cali.^2+y_cali.^2+z_cali.^2);
radius = mean(Y_cali);

[xR,yR,zR] = sphere;

p = p_2;
surf(xR*radius + p(1), yR*radius +p(2), zR*radius +p(3), 'FaceAlpha',0.4,'EdgeColor','none') 
hold on
plot3(x_cali, y_cali, z_cali,'ro')
xlabel('X(cm)')
ylabel('Y(cm)')
zlabel('Z(cm)')
%%

