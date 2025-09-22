clear; clc
data1 = load('2mag_disturb_only.txt');
data1 = data1';
disturb_mag = data1(7:9,:);

data2 = load('const_disturb_IMU.txt');
ref_mag = data2(7:9,:);

for i = 1:size(disturb_mag,2)
    disturb_amp(i) = norm(disturb_mag(:,i));
end
for i = 1:size(ref_mag,2)
    ref_amp(i) = norm(ref_mag(:,i));
end

plot(disturb_amp)
hold on
plot(ref_amp)

range1 = 422:961;
range2 = 165:592;
L1 = size(disturb_mag(range1),2);
L2 = size(ref_mag(range2),2);
distb_sync = resample(disturb_amp(range1), L2, L1);

Ts = 0.03;
times = Ts*(1:size(distb_sync,2));
figure
plot(times, distb_sync)
hold on
plot(times,ref_amp(range2))
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

