%%% IK function written by Aayush Attri(as Research  Intern at
%%%Biorobotics Lab, Carnegie Mellon University)
%%% Last edited on 7 September 2016
%%%Inverse Kinemtics function which returns the target joint angles

%%
 function targ_angles = IK(target,numMod,T_Tail,options)
      x0 = zeros(1,numMod); %starting point as a zero matrix of required modules
     targ_angles = optimizer(x0,options,target,T_Tail); %calling the optimizer 
end

%%
function targ_angles = optimizer(x0,options,target,T_Tail)
    targ_angles = lsqnonlin(@objective,x0,[],[],options); %optimiser
    
    function error = objective(targ_angles)
        TM = FK(targ_angles, T_Tail);
        test = [TM(1:3,3)];
        error = target - test;
    end

end


