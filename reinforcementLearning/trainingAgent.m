% Speedup options
useFastRestart = true;
useGPU = true;
useParallel = true;


% Create the observation info
numObs = 43;
observationInfo = rlNumericSpec([numObs 1]);
observationInfo.Name = 'observations';

% create the action info
numAct = 10;
actionInfo = rlNumericSpec([numAct 1],'LowerLimit',-1,'UpperLimit', 1);
actionInfo.Name = 'foot_torque';

% Environment
mdl = 'Hector_Simulation';
load_system(mdl);
blk = [mdl,'/RL Agent'];
env = rlSimulinkEnv(mdl,blk,observationInfo,actionInfo);
%env.ResetFcn = @(in)walkerResetFcn(in,upper_leg_length/100,lower_leg_length/100,h/100,'2D'); 
if ~useFastRestart
   env.UseFastRestart = 'off';
end

%% CREATE NEURAL NETWORKS
createNeuralNetworks;

%% CREATE AND TRAIN AGENT
createNeuralNetworksOptions;
agent = rlDDPGAgent(actor,critic,agentOptions);
trainingResults = train(agent,env,trainingOptions)

%% SAVE AGENT
reset(agent); % Clears the experience buffer
curDir = pwd;
saveDir = 'savedAgents';
cd(saveDir)
save(['trainedAgent_2D_' datestr(now,'mm_DD_YYYY_HHMM')],'agent','trainingResults');
cd(curDir)

% Walking Robot -- DDPG Agent Training Script (2D)
% Copyright 2019 The MathWorks, Inc.