clear; clc
data1 = load('moving_disturb_only_IMU.txt');
disturb_mag = data1(7:9,:);
acc = data1(1:3,:)';
load('ref_mag.mat');

for i = 1:size(disturb_mag,2)
    disturb_amp(i) = norm(disturb_mag(:,i));
end

plot(disturb_amp)
hold on
plot(ref_mag)

range1 = 73:626;
L1 = size(range1,2);
L2 = size(ref_mag,2);
disturb_sync = resample(disturb_amp(range1), L2, L1);

Ts = 0.03;
times = Ts*(1:size(ref_mag,2));
figure
plot(times, disturb_sync)
hold on
plot(times,ref_mag)
legend('Disturbance', 'Reference')
set(gca, 'Fontsize', 12)
xlabel('Time (s)')
ylabel('Flux (G)')

% norm_N2S = (distb_sync(2:end)-distb_sync(1:end-1))./(ref_amp(range2(2:end))-ref_amp(range2(1:end-1)));
figure
plot(distb_sync(2:end)-distb_sync(1:end-1))
hold on
plot(ref_amp(range2(2:end))-ref_amp(range2(1:end-1)))


%%
for i = 1:3
    disturb_mag_sync(i,:) = resample(disturb_mag(i,:), L2, L1);
    N2S(i,:) = disturb_mag_sync(i,:)./ref_mag(i,:);
end
figure
plot(N2S')
%%

