clear; clc
data1 = load('rot1_Euler.txt');
plot(data1')
ylim([-0.6 0.8])
xlabel('sample')
ylabel('Euler angle (rad)')

%%
data2 = load('rot1_loc_mag.txt');
figure
plot(data2')
ylim([-14 6])
xlabel('sample')
ylabel('location (cm)')
legend('x', 'y', 'z')