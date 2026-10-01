function metrics = compute_control_metrics(simOut,par)
%COMPUTE_CONTROL_METRICS Common tracking and vibration metrics.
%   RMSE is reported for the joint angle theta and flexible-tip angle
%   gamma. Settling requires both errors to remain inside a 2% band.

arguments
    simOut Simulink.SimulationOutput
    par (:,1) double
end

stateTs = simOut.logsout.get("state").Values;
torqueTs = simOut.logsout.get("torque").Values;
t = stateTs.Time(:);
x = squeeze(stateTs.Data);
if size(x,1) ~= numel(t)
    x = x.';
end

theta = x(:,1);
eta1 = x(:,3);
eta2 = x(:,5);
thetaReference = par(19);
alpha = par(9)*eta1 + par(10)*eta2;
gamma = theta + alpha;

jointError = theta - thetaReference;
tipError = gamma - thetaReference;
settlingBand = max(0.02*abs(thetaReference),deg2rad(0.25));
lastOutside = find(abs(jointError) > settlingBand | ...
    abs(tipError) > settlingBand,1,"last");
if isempty(lastOutside)
    settlingTime = 0;
elseif lastOutside == numel(t)
    settlingTime = inf;
else
    settlingTime = t(lastOutside+1);
end

tau = squeeze(torqueTs.Data);
tau = tau(:);
tauTime = torqueTs.Time(:);

metrics = struct;
metrics.JointAngleRMSEDeg = rad2deg(sqrt(mean(jointError.^2)));
metrics.TipAngleRMSEDeg = rad2deg(sqrt(mean(tipError.^2)));
metrics.SettlingTimeS = settlingTime;
metrics.ControlEffortN2m2s = trapz(tauTime,tau.^2);
metrics.TorqueRMSNm = sqrt(trapz(tauTime,tau.^2) / ...
    max(tauTime(end)-tauTime(1),eps));
metrics.PeakVibrationDeg = rad2deg(max(abs(alpha)));

fprintf("\nController performance metrics\n");
fprintf("  Joint-angle RMSE : %.4f deg\n",metrics.JointAngleRMSEDeg);
fprintf("  Tip-angle RMSE   : %.4f deg\n",metrics.TipAngleRMSEDeg);
if isfinite(metrics.SettlingTimeS)
    fprintf("  Settling time    : %.4f s\n",metrics.SettlingTimeS);
else
    fprintf("  Settling time    : Inf (not settled)\n");
end
fprintf("  Control effort   : %.4f N^2 m^2 s\n", ...
    metrics.ControlEffortN2m2s);
fprintf("  RMS torque       : %.4f N m\n",metrics.TorqueRMSNm);
fprintf("  Peak vibration   : %.4f deg\n\n",metrics.PeakVibrationDeg);
end
