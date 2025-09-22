clear; clc;

data = load('center_spot_delay0.5_LIS.txt');

M_bar = mean(data);
M_std = std(data);
plot(data)