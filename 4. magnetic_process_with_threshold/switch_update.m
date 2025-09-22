clear; clc
T_end = 5;
X = 0:0.1:T_end;
V = 0.3;
y = V + sin(2*pi/T_end*X);
plot(X,y)
hold on

scatter(0:4, y(1:10:41), 'g', 'LineWidth', 4)
hold on
scatter(0.4:1:4.4, y(5:10:45), 'm', 'LineWidth', 4)

%  0:T_end-1
for i = 0:T_end-1
    plot( X(10*i+1:10*i+5), y(i*10+1 : i*10+5), 'r', 'LineWidth', 2)
    hold on
    plot([X(10*i+5) X(10*i+10)] ,[y(i*10+5) y(i*10+5)], 'r', 'LineWidth', 2 )
end

legend('Actual Disturb','Switch off', 'Switch on', 'Measured Disturb')
xlabel('Time (s)')
ylabel('Magnetic Strength (G)')
