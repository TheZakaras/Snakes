%%% FK function written by Aayush Attri(as Research  Intern at
%%%Biorobotics Lab, Carnegie Mellon University)
%%% Last edited on 7 September 2016
%%% FK with the coordinate system as described by HebiKinematics
%%% Resultant homogenous transformation matrix is wrt the tail module


function T = FK(angles,T_Tail)
    numMod = length(angles);
    T = T_Tail; %declaration of the final homogenous transformation matrix
    for i = 1:numMod
        T = T*HebiLinkFK(angles(i)); 
    end

end

function T = HebiLinkFK(angle)
    h1 = 0.0366; %length of the first part of the module in meters
    h2 = 0.0273; %length of the second part of the module in meters
    
    T1 = [1 0 0 0;0 1 0 0;0 0 1 h1;0 0 0 1]; %translation of h1 in z
%   T2 = [cos(angle) 0 sin(angle) 0;0 1 0 0;-sin(angle) 0 cos(angle) 0;0 0 0 1];%rotation of angle(i) about y
    T2 = [cos(-angle) 0 sin(-angle) 0;0 1 0 0;-sin(-angle) 0 cos(-angle) 0;0 0 0 1];%
    T3 = [1 0 0 0;0 1 0 0;0 0 1 h2;0 0 0 1];%translation of h2 in z
    T4 = [cos(-pi/2) -sin(-pi/2) 0 0;sin(-pi/2) cos(-pi/2) 0 0;0 0 1 0;0 0 0 1];%rotation of pi/2 cw about z
    
    T = T1*T2*T3*T4; %homogenous transformation matrix for one module
end