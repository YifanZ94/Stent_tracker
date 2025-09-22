clear; clc
%%
files = dir('*.txt') ;   
N = length(files) ;
figure
titles = {'No disturb', 'One object', 'Two objects','Three objects','Four objects'};

for test_num = 1:N-1
    
    thisfile = files(test_num).name ;
    data = load(thisfile);
    mag = data(:,1:3);
    env = data(:,4:6);
    I = data(:,7)/1000;
    net_mag = mag-env;
    mag_amp = sqrt(net_mag(:,1).^2+net_mag(:,2).^2+net_mag(:,3).^2);
    
    I = I*0.084/0.3;
    subplot(3, 2, test_num)
    scatter(I, mag_amp)
    hold on
    
    X = linspace(min(min(I)), max(max(I)), 10);
    f = fit(I, mag_amp, 'poly1');
    P(test_num,1) = f.p1;
    P(test_num,2) = f.p2;
    plot(X, f.p1*X+f.p2, '--')
    
    xlabel('IB_0/I_0')
    ylabel('Flux (G)')
    title(titles(test_num))
end

%%
% data = load('No_current_mag.txt');
% mag = data(:,1:3);
% env = data(:,4:6);
% I = data(:,7)/1000;
% mag_amp = rms(mag-env, 2);
% figure
% scatter(I, mag_amp)