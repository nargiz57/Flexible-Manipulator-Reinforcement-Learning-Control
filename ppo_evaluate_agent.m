function metrics = ppo_evaluate_agent(agent,env,maxSteps,par,thetaD)
% Deterministically score one direct-torque PPO policy.
arguments
    agent
    env
    maxSteps (1,1) double {mustBeInteger,mustBePositive}
    par
    thetaD (1,1) double = pi/6
end

agent.UseExplorationPolicy = false;
experience = sim(env,agent,rlSimulationOptions(MaxSteps=maxSteps));

obs = squeeze(experience.Observation.observations.Data);
if isvector(obs)
    obs = reshape(obs,12,[]);
end
actions = squeeze(experience.Action.torque.Data);
actions = actions(:);
rewards = experience.Reward.Data;
rewards = rewards(:);

n = size(obs,2);
nSteps = numel(rewards);
Ts = par(32);
eGamma = obs(1,:)*(pi/6);
gammaDot = obs(2,:);
eTheta = obs(3,:)*(pi/6);
eta1 = obs(5,:)*0.010;
eta1Dot = obs(6,:)*sqrt(par(13)/par(11))*0.010;
eta2 = obs(7,:)*0.0025;
eta2Dot = obs(8,:)*sqrt(par(14)/par(12))*0.0025;
E1 = 0.5*par(11)*eta1Dot.^2+0.5*par(13)*eta1.^2;
E2 = 0.5*par(12)*eta2Dot.^2+0.5*par(14)*eta2.^2;
[~,~,~,~,~,E1Scale,E2Scale] = ...
    ppo_modal_quantities(zeros(6,1),par);
normalizedEnergy = E1/E1Scale+E2/E2Scale;

direction = sign(thetaD);
if direction == 0
    direction = 1;
end
overshoot = max([0,direction*eGamma]);
torqueVariation = sum(abs(diff(actions)));

iaeGamma = Ts*sum(abs(eGamma));
iaeTheta = Ts*sum(abs(eTheta));
integratedModalEnergy = Ts*sum(normalizedEnergy);
completionPenalty = 100*max(0,1-nSteps/maxSteps);

lastCount = min(n,max(1,round(1/Ts)));
lastIdx = (n-lastCount+1):n;
finalGammaError = mean(abs(eGamma(lastIdx)));
finalThetaError = mean(abs(eTheta(lastIdx)));
finalTrackingPenalty = ...
    2.0*(rad2deg(finalGammaError)+rad2deg(finalThetaError));

score = ...
    4*iaeGamma+ ...
    2*iaeTheta+ ...
    3*integratedModalEnergy+ ...
    2*rad2deg(overshoot)+ ...
    0.2*torqueVariation+ ...
    finalTrackingPenalty+ ...
    completionPenalty;

trackingBand = max(0.02*abs(thetaD),deg2rad(0.25));
outside = find(abs(eGamma)>trackingBand | abs(eTheta)>trackingBand,1,"last");
if isempty(outside)
    settlingTime = 0;
elseif outside == n
    settlingTime = inf;
else
    settlingTime = outside*Ts;
end

firstInside = find(abs(eGamma)<=trackingBand & abs(eTheta)<=trackingBand,1);
if isempty(firstInside)
    residualTipVibration = inf;
else
    alpha = eGamma-eTheta;
residualTipVibration = max(abs(alpha(firstInside:end)));
end

if isfinite(settlingTime)
    settlingPenalty = 1.5*settlingTime;
else
    settlingPenalty = 20;
end
if isfinite(residualTipVibration)
    vibrationPeakPenalty = 2*rad2deg(residualTipVibration);
else
    vibrationPeakPenalty = 20;
end

score = score+settlingPenalty+vibrationPeakPenalty;

metrics = struct;
metrics.Score = score;
metrics.TotalReward = sum(rewards);
metrics.CompletedSteps = nSteps;
metrics.MaxSteps = maxSteps;
metrics.IAEGamma = iaeGamma;
metrics.IAETheta = iaeTheta;
metrics.IntegratedModalEnergy = integratedModalEnergy;
metrics.OvershootRad = overshoot;
metrics.TorqueVariation = torqueVariation;
metrics.FinalGammaErrorRad = finalGammaError;
metrics.FinalThetaErrorRad = finalThetaError;
metrics.SettlingTime = settlingTime;
metrics.ResidualTipVibrationRad = residualTipVibration;
metrics.PeakGammaRate = max(abs(gammaDot));
metrics.PassedNominalTracking = ...
    nSteps == maxSteps && ...
    finalGammaError <= deg2rad(0.5) && ...
    finalThetaError <= deg2rad(0.5) && ...
    overshoot <= deg2rad(0.5) && ...
    isfinite(settlingTime) && ...
    metrics.TotalReward > 0;
end
