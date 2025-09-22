clear; clc

%% dynamic model
quat_iteration = @(omega,Ts)    expm([0 -omega(1) -omega(2) -omega(3);
        omega(1) 0 omega(3) -omega(2);
        omega(2) -omega(3) 0 omega(1);
        omega(3) omega(2) -omega(1) 0]*Ts/2);
    
Rot_by_quat = @(q)   [q(1)^2+q(2)^2-q(3)^2-q(4)^2, 2*(q(2)*q(3)-q(1)*q(4)), 2*(q(1)*q(3)+q(2)*q(4));
         2*(q(2)*q(3)+q(1)*q(4)), q(1)^2-q(2)^2+q(3)^2-q(4)^2, 2*(q(3)*q(4)-q(1)*q(2));
         2*(q(2)*q(4)-q(1)*q(3)), 2*(q(1)*q(2)+q(3)*q(4)), q(1)^2-q(2)^2-q(3)^2+q(4)^2];

Rot_by_Eulers = @(roll,pitch,yaw) [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)]* ...
                                [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)]*...
                                [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1];

Euler_by_quat = @(q) [atan2(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2))
                      asin(2*(q(1)*q(3)- q(4)*q(2)));  
                      atan2(2*(q(1)*q(4)+q(2)*q(3)), 1-2*(q(3)^2+q(4)^2));];  % X, Z reversed
                  
Euler_by_acc = @(a) [atan2(a(2),a(3));
                    atan(-a(1)/sqrt(a(2)^2 + a(3)^2));
                    0];

quat_iteration_2 = @(omega,Ts)    eye(4)+ [0 -omega(1) -omega(2) -omega(3);
                omega(1) 0 omega(3) -omega(2);
                omega(2) -omega(3) 0 omega(1);
                omega(3) omega(2) -omega(1) 0]*Ts/2;

Rx = @(roll) [1,0,0; 0,cos(roll),-sin(roll); 0,sin(roll),cos(roll)];
Ry = @(pitch) [cos(pitch),0,sin(pitch); 0,1,0; -sin(pitch),0,cos(pitch)];
Rz = @(yaw) [cos(yaw),-sin(yaw),0; sin(yaw),cos(yaw),0; 0,0,1];

%% rot matrix by acc       
roll = pi/4;
pitch = pi/5;
yaw = pi/6;

a = [7;8;9];
g = [0;0;1];

R_act = Rz(yaw)*Ry(pitch)*Rx(roll);
R_pas = Rx(roll)'*Ry(pitch)'*Rz(yaw)';

%%
g_reading = R_pas*g;
Eulers = Euler_by_acc(g_reading);
g_read_to_origin = R_act*g_reading;
g_xy_only = Ry(Eulers(2))*Rx(Eulers(1))*g_reading;
g_yx_only = Rz(Eulers(3))*Rx(Eulers(1))*Ry(Eulers(2))*g_reading;

%%
a_zyx_reading = R_pas*a;
a_z = Rz(yaw)'*a;
a_xy_correct = Ry(Eulers(2))*Rx(Eulers(1))*a_zyx_reading;

% a_reading = R_pas*a;
%%
% a = g_reading;
a1 = R_act*a;
a2 = Rz(yaw)*  (Ry(pitch)*Rx(roll)*a);
a3 = Rz(yaw)*  (Ry(pitch)*  (Rx(roll)*a) );

a4 = Rx(roll)* Ry(pitch) *a;
a5 = (Ry(pitch) *Rx(roll))* a;

