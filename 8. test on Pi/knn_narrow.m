function [position,position_ind_in_2dMap] = knn_narrow(magMap_norm, locMap, location_ind, submap_range, measure_R)
% location_ind = I(end);
% submap_range = 10;

Bx = magMap_norm{1}; 
By = magMap_norm{2};
Bz = magMap_norm{3};
Px = locMap{1};
Py = locMap{2};
Pz = locMap{3};
Xsize = size(Bx,1);
Ysize = size(Bx,2);
Zsize = size(Bx,3);

Zind = ceil(location_ind/(Xsize*Ysize));
Yind = ceil((location_ind-(Zind-1)*(Xsize*Ysize))/Xsize);
Xind = round(location_ind-(Zind-1)*(Xsize*Ysize) - (Yind-1)*(Xsize));

X_LB = max(Xind-submap_range,1);
X_UB = min(Xind+submap_range,Xsize);
Y_LB = max(Yind-submap_range,1);
Y_UB = min(Yind+submap_range,Ysize);
Z_LB = max(Zind-submap_range,1);
Z_UB = min(Zind+submap_range,Zsize);

B_2d = [reshape(Bx(X_LB:X_UB, Y_LB:Y_UB, Z_LB:Z_UB),[],1),reshape(By(X_LB:X_UB, Y_LB:Y_UB, Z_LB:Z_UB),[],1),reshape(Bz(X_LB:X_UB, Y_LB:Y_UB, Z_LB:Z_UB),[],1)];
Loc_2d = [reshape(Px(X_LB:X_UB, Y_LB:Y_UB, Z_LB:Z_UB),[],1),reshape(Py(X_LB:X_UB, Y_LB:Y_UB, Z_LB:Z_UB),[],1),reshape(Pz(X_LB:X_UB, Y_LB:Y_UB, Z_LB:Z_UB),[],1)];

I = knnsearch(B_2d, measure_R, 'K', 1);
position = Loc_2d(I,:)';
position_ind_in_2dMap = abs((position(1)-Px(1,1,1))/(Px(2,1,1)-Px(1,1,1)))+1 ...
        +(abs((position(2)-Py(1,1,1))/(Py(1,2,1)-Py(1,1,1)))+1)*Xsize ...
        +(abs((Pz(1,1,1)-position(3))/(Pz(1,1,2)-Pz(1,1,1)))+1)*Xsize*Ysize;
            
end