function reward = ppo_reward(x,xPrev,tauApplied,tauAppliedPrev,done,par)
%#codegen

thetaD = par(19);
tauMax = par(22);
Ts = par(32);
stage = par(33);
discount = exp(-Ts/2.5);

[~,gamma,gammaDot,E1,E2,E1Scale,E2Scale] = ...
    ppo_modal_quantities(x,par);
[~,gammaPrev,~,E1Prev,E2Prev,~,~] = ...
    ppo_modal_quantities(xPrev,par);

eGamma = gamma-thetaD;
eTheta = x(1)-thetaD;
eGammaPrev = gammaPrev-thetaD;
eThetaPrev = xPrev(1)-thetaD;

zGamma = eGamma/(3*pi/180);
zTheta = eTheta/(3*pi/180);
zGammaDot = gammaDot/0.25;
zE1 = E1/E1Scale;
zE2 = E2/E2Scale;
zTau = tauApplied/tauMax;
zDeltaTau = (tauApplied-tauAppliedPrev)/0.5;

direction = sign(thetaD);
if direction == 0
    direction = 1;
end
zOvershoot = max(direction*eGamma,0)/(pi/180);

% Brake before the target instead of waiting for an overshoot to occur.
% The allowed closing speed decreases with the remaining tip-angle error.
closingSpeed = direction*gammaDot;
allowedClosingSpeed = 0.04 + 1.5*abs(eGamma);
zExcessClosingSpeed = max(closingSpeed-allowedClosingSpeed,0)/0.08;
brakingGate = exp(-(eGamma/(8*pi/180))^2);

vibrationWeight = 1.0;
tipRateWeight = 0.35;
progressWeight = 5.0;
if stage == 1
    vibrationWeight = 0.10;
    tipRateWeight = 0.05;
    progressWeight = 25.0;
end

rhoGamma = boundedSquare(zGamma);
rhoTheta = boundedSquare(zTheta);
rhoGammaDot = boundedSquare(zGammaDot);
rhoE1 = boundedSquare(zE1);
rhoE2 = boundedSquare(zE2);
rhoTau = boundedSquare(zTau);
rhoDeltaTau = boundedSquare(zDeltaTau);
rhoOvershoot = boundedSquare(zOvershoot);
rhoExcessClosingSpeed = boundedSquare(zExcessClosingSpeed);

nearTargetWeight = 0.01 + 0.07*exp(-(eGamma/(5*pi/180))^2);
cost = ...
    1.6*rhoGamma + ...
    1.0*rhoTheta + ...
    tipRateWeight*rhoGammaDot + ...
    vibrationWeight*(1.2*rhoE1 + 0.7*rhoE2) + ...
    0.03*rhoTau + ...
    nearTargetWeight*rhoDeltaTau + ...
    1.6*rhoOvershoot + ...
    1.2*brakingGate*rhoExcessClosingSpeed;

phiCurrent = potential(eGamma,eTheta,E1,E2,E1Scale,E2Scale);
phiPrevious = potential( ...
    eGammaPrev,eThetaPrev,E1Prev,E2Prev,E1Scale,E2Scale);
progressReward = progressWeight*(discount*phiCurrent-phiPrevious);

approachVelocityReward = 0.0;
if stage == 1
    approachRate = -sign(eGamma)*gammaDot;
    errorGate = min(abs(eGamma)/(10*pi/180),1.0);
    approachVelocityReward = ...
        1.25*errorGate*tanh(approachRate/0.12);
end

holdReward = 2.0*exp( ...
    -(eGamma/(0.75*pi/180))^2 ...
    -(eTheta/(0.75*pi/180))^2 ...
    -(gammaDot/0.03)^2 ...
    -E1/max(par(13)*(0.0007)^2,1e-12) ...
    -E2/max(par(14)*(0.00015)^2,1e-12));

reward = 0.25-cost+holdReward+progressReward+approachVelocityReward;
if done
    reward = reward-5.0;
end
reward = min(max(reward,-5.0),5.0);
end

function value = boundedSquare(z)
z2 = z*z;
value = z2/(1.0+z2);
end

function value = potential(eGamma,eTheta,E1,E2,E1Scale,E2Scale)
value = -( ...
    1.5*boundedSquare(eGamma/(10*pi/180)) + ...
    0.7*boundedSquare(eTheta/(10*pi/180)) + ...
    0.8*boundedSquare(E1/(4*E1Scale)) + ...
    0.4*boundedSquare(E2/(4*E2Scale)));
end
