clear; clc
load('Square_X40_Y20_Z15_reso0.5_T.mat');
file = {'H6.5_LIS.txt';
    'H8_LIS.txt';
    'H10_LIS.txt';
    'H11_LIS.txt';
    'H13_LIS.txt';
    'H15_LIS.txt';
    'H17_LIS.txt'};
magnitude_map = sqrt(BX.^2 + BY.^2 + BZ.^2);
H = {6.5,8,10,11,13,15,17};
Z_ind_list = {4,7,11,13,17,21,25};
Y_ind_list = {-1,-1,-1,1,0,0,-1};
X_ind_list = {1,-1,0,1,-1,0,0};
x = linspace(-15,15,31);

figure
% 1:size(file,1)
for test_group = [1,4,7]
    data = load(file{test_group});
    magnitude_test = sqrt(data(:,1).^2 + data(:,2).^2 + data(:,3).^2);
    
    Z_index = Z_ind_list{test_group};    % 7, 13, 21
    Y_index = 21 + Y_ind_list{test_group};
    X_index = linspace(11,71,size(data,1)) + X_ind_list{test_group};
    distance(:,test_group) = sqrt(H{test_group}^2 + x.^2);
    C_magnitude(:,test_group) = magnitude_test./magnitude_map(X_index,Y_index,Z_index);

    plot(x,C_magnitude(:,test_group), '-x')
    hold on
    xlabel('distance(cm)')
    ylabel('test/model ratio of magnitude')
end

% ylim([0.6 1.4])
% legend('H=6.5', 'H=11', 'H=18')
%% curve fitting
D = reshape(distance,[],1);
R = reshape(C_magnitude,[],1);
R_N = normalize(R, 'range');

figure
% scatter(D,R)
% xlabel('distance(cm)')
% ylabel('test/model ratio')

f= fit(D,R,'poly4');
plot(f,D,R)
xlabel('distance(cm)')
ylabel('test/model ratio')

%% transform the mag field
% load('Square_X40_Y20_Z15_reso0.2.mat')
% ratio_d = @(d) f.p1*d^4 + f.p2*d^3 + f.p3*d^2 + f.p4*d + f.p5;
% 
% for map_x = 1:size(MapX,1)
%     for map_y = 1:size(MapX,2)
%         for map_z = 1:size(MapX,3)
%             D = sqrt(MapX(map_x,map_y,map_z)^2+ MapY(map_x,map_y,map_z)^2+ MapZ(map_x,map_y,map_z)^2);
%             M = ratio_d(D);
%             BX(map_x,map_y,map_z) = BX(map_x,map_y,map_z)*M;
%             BY(map_x,map_y,map_z) = BY(map_x,map_y,map_z)*M;
%             BZ(map_x,map_y,map_z) = BZ(map_x,map_y,map_z)*M;
%         end
%     end
% end
% disp('done')