%%%Head Stability System written by Aayush Attri( as Research  Intern at
%%%Biorobotics Lab, Carnegie Mellon University)
%%% Last edited on 7 September 2016
%%% This script helps the head module of the snake robot point in a
%%% pre-defined direction in the world frame

load('offsets.mat');%trustabilites and offsets of the gyros and accelerometers
snake = HebiLookup.newConnectedGroupFromName('*','SA002');
snakeData = setupSnakeData( 'SEA Snake', snake.getInfo.numModules);


numMod = snake.getInfo.numModules;
baseMod = 6; % number of modules taking part in the IK

%% Complementary Filter Initialisation
CF = ComplementaryFilter(snakeData,'accelOffsets',accelOffsets(:,1:numMod),...
        'gyroOffsets',gyroOffsets(:,1:numMod),'gyrosTrustability',...
        gyrosTrustability(1:numMod),'accTrustability',accTrustability(1:numMod));
    
fbk = snake.getNextFeedback();
CF.update(fbk);

pause(0.25); 

fbk = snake.getNextFeedback();
CF.update(fbk);

%% User defined direction vector in the gravity frame
targ_dir_head = [-1 0 0 1]; % [ i j k 1]
targ_dir_head = transpose(targ_dir_head);   
dir = targ_dir_head(1:3);
target = [dir/norm(dir)]; % normalized direction vector

%% Setting up the snake 
ones_n = ones(1,numMod);  %gains exacly taken from Julian's demo code

gain = snake.getGains();
gain.controlStrategy = ones_n*4;
%gain.positionKp = ones_n*6; 
gain.positionKp = ones_n*10;
gain.positionKi = ones_n*.01; 
gain.positionKd = ones_n*1; 
gain.torqueKp= ones_n*1;%1
gain.torqueKi= ones_n*0;
gain.torqueKd= ones_n*.1;
gain.velocityKp = ones_n*1;
snake.set('gains',gain);
pause(0.5);

cmd = CommandStruct();
angles = zeros(1,numMod);
fin_ang = zeros(1,numMod);
fbk = [];
fbk_angles = [];

%% Options and tolerances for the optimizer
tol = 1e-2;  %to set the function tolerance for the optimiser. 
tox = 1e-2;
iter = 5;
eval = 0;
options = optimset();
options = optimset(options, 'TolFun', tol,'TolX',tox,'MaxIter',iter,'Display','off');
%% 
while true

    fbk = snake.getNextFeedback();
    CF.update(fbk); %updating the CF
    fbk_angles = fbk.position;
    fbk_head = headMod.getNextFeedback();
    fin_ang = zeros(1,numMod)*nan;
    T_Tail = CF.getInGravity('tail');%Tail frame with respect to gravity
    goal_angles = IK(target,baseMod,T_base,options);
    alpha = 0;
    angles = fbk_angles((16-baseMod+1):16) * alpha + goal_angles*(1-alpha);
    fin_ang((16-baseMod+1):16) = angles;
    cmd.position = fin_ang;
    snake.set(cmd);
    pause(0.01 );
end
