function rollAng = getRollAng(t,numMod)
    

%% Slithering parameters
% spat_freq = 0.8;
% temp_freq= pi;
% A_even = 0.7;
% A_odd = 0.7*0.6;
% delta = 0;
% beta_odd = 0;
% beta_even = 0;
% taper = .6;
%%

%% Sidewinding parameters
% spat_freq = 0.5;
% temp_freq = 4;
% A_even = .9;
% A_odd = .5;
% delta = pi/4; % right
% % delta = -pi/4; % left
% beta_odd = 0;
% beta_even = 0;

%% Rolling parameters
spat_freq = 0;
temp_freq= 7;
A_even = 0.4;
A_odd = 0.4;
% delta = -pi/2;%goes right
delta = pi/2;%for left
beta_odd = 0;
beta_even = 0;
% dt = 1;
% taper = 0.6;

reversals = [];
for reversal=1:ceil(numMod/2)
   negOrPos = mod(reversal,2)*2-1;
   reversals = [reversals,negOrPos,negOrPos];
end
reversals = reversals(1:numMod);

%% Generating angles
for i = 1:(numMod)  % +2 for 5th and 6th module
    %       for i = 1 : numMod
    if mod(i,2)
        %           slither_angles(i) = beta_odd + A_odd*sin(spat_freq*i*0.4 + temp_freq*t + delta);
        rollAng(i) = beta_odd + A_odd*sin(spat_freq*i + temp_freq*t + delta);
    else
        %             slither_angles(i) = beta_even + A_even*sin((spat_freq*i + 2*temp_freq*t));
        rollAng(i) = beta_even + A_even*sin((spat_freq*i + temp_freq*t));
        
    end
    %        slither_angles(i) = slither_angles(i)*(numMod - taper*abs(i-19))/numMod;
end

rollAng = changeUnifiedToSEA(reversals,rollAng);