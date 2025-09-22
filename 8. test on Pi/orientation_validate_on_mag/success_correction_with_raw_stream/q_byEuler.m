function q = q_byEuler(Eulers) 
%% in body 3-2-1 sequence
    Eulers = Eulers/2;

%     q = [cos(Eulers(1))*cos(Eulers(2))*cos(Eulers(3))+sin(Eulers(1))*sin(Eulers(2))*sin(Eulers(3));
%      cos(Eulers(1))*cos(Eulers(2))*sin(Eulers(3))-sin(Eulers(1))*sin(Eulers(2))*cos(Eulers(3));
%      cos(Eulers(1))*sin(Eulers(2))*cos(Eulers(3))+sin(Eulers(1))*cos(Eulers(2))*sin(Eulers(3));
%      sin(Eulers(1))*cos(Eulers(2))*cos(Eulers(3))-cos(Eulers(1))*sin(Eulers(2))*sin(Eulers(3))]; 
 
     q = [cos(Eulers(1))*cos(Eulers(2))*cos(Eulers(3)) + sin(Eulers(1))*sin(Eulers(2))*sin(Eulers(3));
     cos(Eulers(2))*cos(Eulers(3))*sin(Eulers(1)) - sin(Eulers(2))*sin(Eulers(3))*cos(Eulers(1));
     cos(Eulers(1))*sin(Eulers(2))*cos(Eulers(3)) + sin(Eulers(1))*cos(Eulers(2))*sin(Eulers(3));
     sin(Eulers(3))*cos(Eulers(2))*cos(Eulers(1)) - cos(Eulers(3))*sin(Eulers(2))*sin(Eulers(1))]; 

%% 
end