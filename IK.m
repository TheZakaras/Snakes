%%%IK with direction of cartesian coordinates

function targ_angles = IKf(target,numMod,T_Tail,curr_ang,options,ang_cost,pos_cost)

x0 = zeros(1,numMod); %starting point as a zero matrix of required modules
% x0 = curr_ang;
targ_angles = optimiser(x0,options,target,T_Tail,numMod,ang_cost,curr_ang,pos_cost);
end
%%


function targ_angles = optimiser(x0,options,target,T_Tail,numMod,ang_cost,curr_ang,pos_cost)
% for i = 1:numMod
%     ub(i) = 1.77*((numMod - i +1)/numMod);
%     lb(i) =-1.77*((numMod - i +1)/numMod);
% end
% ub = fliplr(ub);
% lb = fliplr(lb);

% ub = 1.77*ones(1,numMod);
% lb = -1.77*ones(1,numMod);
% ub(1:2) = curr_ang(1:2) + ang_grad(1:2);
% lb(1:2) = curr_ang(1:2) - ang_grad(1:2);
    
    ub = 1.77*ones(1,numMod);
    lb = -1.77*ones(1,numMod);
targ_angles = lsqnonlin(@objective,x0,lb,ub,options); %optimiser

    function error = objective(targ_angles)
        T = eye(4);
        T_mid = FK2(targ_angles,T);
        T_Tail(3,4) = 0; % so that the CoG does not vary
        TM = T_Tail*T_mid;
        test = [TM(3,4);TM(1:3,3)];
        error = target - test;
        
        error(5) = ang_cost(1)*(targ_angles(1) - curr_ang(1));
        error(6) = ang_cost(2)*(targ_angles(2) - curr_ang(2));
        error(1) = pos_cost*error(1);

    end

end




