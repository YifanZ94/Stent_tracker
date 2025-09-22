load('Mag_Current.mat')
% clear; clc
% data = load('1_mag.txt');
C = 5.88;
for i = 1:5
    for j = 1:5
        I(i, j) = data((i-1)*5+j,4);
        mag_amp(i, j) = norm(data((i-1)*5+j,1:3))*C;
    end
end
I = I/1000;
%%
marker = ['o','+','x','*','^'];
color = ['r','g','b','c','m'];
% 
for j = 1:size(I,1)
    scatter(I(j,:),mag_amp(j,:),80,'x',color(j),'LineWidth',2)
    hold on
end
xlabel('I (A)')
ylabel('Flux (G)')
legend('no disturb','disturb 1','disturb 2','disturb 3','disturb 4')

%%
X = linspace(min(min(I)), max(max(I)), 10);
for j = 1:size(I,1)
    f = fit(I(j,:)', mag_amp(j,:)', 'poly1');
    P(j,1) = f.p1;
    P(j,2) = f.p2;
    plot(X, f.p1*X+f.p2, strcat(color(j),'--'))
    hold on
end