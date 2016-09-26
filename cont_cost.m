%%% Gait + head stabilisation 


%% Snake group formation
load('offsets.mat');
snake = HebiLookup.newConnectedGroupFromName('*','SA002');
snakeData = setupSnakeData( 'SEA Snake', snake.getInfo.numModules);


%% Plotting 
% plt = HebiPlotter();

%% Variable Initialisation
numMod = 16;
baseIK = 6; % the first six modules
hybridMod = 2; % number of modules after 4th having dual function
gaitMod  = numMod - baseIK + hybridMod;% modules which perform the gait
kin = HebiKinematics;
snake.setFeedbackFrequency(200);
T_Tail = eye(4);
T_gravity = eye(4);
for i = 1:baseIK % to be used for gravity compensation
    kin.addBody('FieldableElbowJoint');
end

cmd = CommandStruct();
fin_ang = zeros(1,numMod);
fbk_angles = [];
gait_angles = zeros(1,gaitMod);


t = 0;



%% Complementary Filter Initialiation
CF = ComplementaryFilter(snakeData,'accelOffsets',accelOffsets(:,1:numMod),...
        'gyroOffsets',gyroOffsets(:,1:numMod),'gyrosTrustability',...
        gyrosTrustability(1:numMod),'accTrustability',accTrustability(1:numMod));
    
fbk = snake.getNextFeedback();
CF.update(fbk);

pause(0.25); 

fbk = snake.getNextFeedback();
CF.update(fbk);

%% Target vectors
targ_coord_grav = [-0.2 0.26 0.0693 1];
dir = [-1 0 0];
% dir = -1;


dir = transpose(dir);
% target = [targ_coord_grav(1:3)';dir/norm(dir)];
target = [targ_coord_grav(1:3)';dir/norm(dir)];

%% Gains settings
ones_n = ones(1,numMod);  
gain = snake.getGains();
gain.controlStrategy = ones_n*4;
gain.positionKp = ones_n*6;
gain.positionKp((16 - baseIK + 1):16)  = 4; 
gain.positionKi = ones_n*.01; 
gain.positionKd = ones_n*5; %originally 1
gain.positionKd((16 - baseIK + 1):16) = 15;% try with 4 again. 1.5 seems good
gain.torqueKp= ones_n*0.5;
gain.torqueKi= ones_n*0;
gain.torqueKd= ones_n*5;% originally .1
gain.torqueKd((16 - baseIK + 1):16) = 10;
gain.velocityKp = ones_n*1;
snake.set('gains',gain);
pause(0.5);


%% Optimiser options
tol = 1e-2;  %to set the function tolerance for the optimiser.
tox = 1e-2;
iter = 5;
eval = 0;
options = optimset();
options = optimset(options, 'TolFun', tol,'TolX',tox,'MaxIter',iter,'Display'...
    ,'off');
 
%% Cost constants for the optimiser
a = 1; %a>0
b = 1.2;% b > 1
for i = 1:numMod
%     ang_cost = ones(1,baseIK);%ang_grad(baseMod) is the head
%     ang_cost(1:2) = 1; % for 5th and 6th module
    ang_cost(i) = a*b^i;
end

dir_cost = 10;
pos_cost = 50;

%% Initial display after all the settings
display('You can move the snake to the desired orientation');
display('Press any key to start the gait');
pause;
CF.update(snake.getNextFeedback());


%% Test parameters
T_Test = eye(4); %used for testing
% gait_angles = zeros(1,gaitMod);

%% main loop
while t<50
    %% Gait 
    t = t + 0.005;
%     t = 0.005;
    fbk = snake.getNextFeedback();
    CF.update(fbk);
    fbk_angles = fbk.position;
    gait_angles = getRollAng(t,gaitMod);%get Rolling Angles
%     gait_angles = getSlitherAng(t,gaitMod);%get Slithering Angles
    %% Head stabilisation
    T_Tail = CF.getInGravity('tail');% tail wrt world
    T_Head = CF.getInGravity('head')
    %base module indicates the starting point of IK i.e. 6th module
   % T_base = FK2(fbk_angles(1:(numMod - baseIK)),T_Tail); %base wrt world
   T_base = FK2(fbk_angles(1:numMod - baseIK),T_Tail);
%    FK2(fbk.position,T_Tail)
%     par_angles(1) = fbk_angles(numMod - baseIK + 1);
%     par_angles(2) = fbk_angles(numMod - baseIK + 2);
 %   par_angles(3:6) = fbk.position((numMod - baseIK + 3):numMod);
    goal_angles = IKf(target,baseIK,T_base,fbk_angles(numMod-baseIK+1:numMod...
        ),options,ang_cost,pos_cost,dir_cost);
    
    fin_ang((numMod-baseIK+1):numMod) = goal_angles;% IK angles
    fin_ang(1:(numMod-baseIK)) = gait_angles(1:(numMod-baseIK));%gait angles
%     fin_ang(numMod-baseIK +1:numMod-baseIK+2) = goal_angles(1:2); % duality angles
    
    cmd.position = fin_ang;
%     
%     T = eye(4);
%         T_mid = FK2(fin_ang(1:(numMod -baseIK + 2)),T);
%         T_Tail(1:3,4) = 0; % so that the CoG does not move
%         T_Tail(1:3,4) = 0;
%         T_Test = T_Tail*T_mid
%         FK2(fin_ang(numMod-baseIK+3:numMod),T_Test)

%     T_Test = FK2(fin_ang,T_)
    %% Gravity's torque compensation
    gravity = -1*T_base(3,1:3);
    cmd.torque((numMod - baseIK +1):numMod) = kin.getGravCompTorques(fbk...
        .position((numMod - baseIK +1):numMod),gravity);
    snake.set(cmd);
%     plt.plot(cmd.position);
    pause(0.001);
    
%     disp('Feedback position');
%     disp(fbk.position);
%     disp('Gait position');
%     disp(fin_ang);
    

end