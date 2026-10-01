function simIn = reset_ppo_evaluation(simIn,par,thetaD,x0Eval)
%RESET_PPO_EVALUATION Deterministic nominal reset for policy selection.
arguments
    simIn
    par
    thetaD (1,1) double = pi/6
    x0Eval (6,1) double = zeros(6,1)
end

parEval = par;
parEval(19) = thetaD;
parEval(33) = 2;
simIn = setVariable(simIn,"x0",x0Eval);
simIn = setVariable(simIn,"x_prev0",x0Eval);
simIn = setVariable(simIn,"tau_prev0",0.0);
simIn = setVariable(simIn,"par",parEval);
end
