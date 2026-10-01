function simIn = reset_ppo_flexible_manipulator(simIn,par,stage)
%RESET_PPO_FLEXIBLE_MANIPULATOR Apply the three-stage training curriculum.
arguments
    simIn
    par
    stage (1,1) double {mustBeInteger,mustBeInRange(stage,1,3)}
end

thetaD = pi/6;
if stage == 3
    references = (pi/180)*[-40 -30 -20 -10 10 20 30 30 30 40];
    thetaD = references(randi(numel(references)));
end

theta0 = deg2rad(-1.0+2.0*rand);
thetaDot0 = -0.03+0.06*rand;
eta10 = -0.002+0.004*rand;
eta1Dot0 = -0.02+0.04*rand;
eta20 = -0.0004+0.0008*rand;
eta2Dot0 = -0.04+0.08*rand;
x0New = [theta0;thetaDot0;eta10;eta1Dot0;eta20;eta2Dot0];

parNew = par;
parNew(19) = thetaD;
parNew(33) = stage;

if stage == 3
    payloadFactor = 0.85+0.30*rand;
    stiffnessFactor = 0.85+0.30*rand;
    dampingFactor = 0.85+0.30*rand;
    parNew(5) = par(5)*payloadFactor;
    parNew(13:14) = par(13:14)*stiffnessFactor;
    parNew(15:16) = par(15:16)*dampingFactor*sqrt(stiffnessFactor);
end

simIn = setVariable(simIn,"x0",x0New);
simIn = setVariable(simIn,"x_prev0",x0New);
simIn = setVariable(simIn,"tau_prev0",0.0);
simIn = setVariable(simIn,"par",parNew);
end
