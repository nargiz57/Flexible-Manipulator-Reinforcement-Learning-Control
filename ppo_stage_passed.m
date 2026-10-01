function [passed,reason] = ppo_stage_passed(metrics,stage)
% Prevent curriculum advance from a failed policy.
arguments
    metrics (1,1) struct
    stage (1,1) double {mustBeInteger,mustBeInRange(stage,1,3)}
end

completed = metrics.CompletedSteps == metrics.MaxSteps;
tipErrorDeg = rad2deg(metrics.FinalGammaErrorRad);
hubErrorDeg = rad2deg(metrics.FinalThetaErrorRad);
overshootDeg = rad2deg(metrics.OvershootRad);

switch stage
    case 1
        passed = completed ...
            && tipErrorDeg <= 6 ...
            && hubErrorDeg <= 6 ...
            && overshootDeg <= 5;
        target = "complete horizon, <=6 deg final errors and <=5 deg overshoot";
    case 2
        passed = completed ...
            && tipErrorDeg <= 1 ...
            && hubErrorDeg <= 1 ...
            && overshootDeg <= 1 ...
            && metrics.TotalReward > 0;
        target = "complete horizon, <=1 deg errors/overshoot, positive return";
    otherwise
        passed = metrics.PassedNominalTracking;
        target = "nominal publication gates";
end

reason = sprintf( ...
    "Stage %d requires %s. Actual: %d/%d steps, tip %.3f deg, "+ ...
    "hub %.3f deg, overshoot %.3f deg, return %.3f.", ...
    stage,target,metrics.CompletedSteps,metrics.MaxSteps, ...
    tipErrorDeg,hubErrorDeg,overshootDeg,metrics.TotalReward);
end
