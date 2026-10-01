clear; clc; close all;
close_stale_rl_models("assumed_mode_ppo_simulink");

if isfile("trained_mep_ppo.mat")
    S = load("trained_mep_ppo.mat");
else
    error("MEP:NoPublishedPolicy", ...
        "No policy passed the publication gates. "+ ...
        "Do not test ppo_resume_state.mat as a finished controller. "+ ...
        "Run train_ppo_simulink after the exploration fix.");
end

agentObj = S.agent;
agentObj.UseExplorationPolicy = false;
p = S.p;
Ts = S.Ts;
Tf = 10;
% x = [theta; theta_dot; eta1; eta1_dot; eta2; eta2_dot]
x0 = [0; 0; 0; 0; 0; 0];
x_prev0 = x0;
tau_prev0 = 0.0;
par = build_ppo_parameter_vector(p,Ts,2);
referenceDeg = 30;          % desired reference angle
par(19) = deg2rad(referenceDeg);
par(5) = 1.30*par(5);       % payload +30%
par(13:14) = 0.75*par(13:14); % modal stiffness -25%
par(15:16) = 0.70*par(15:16); % modal damping -30%
assignin("base","agentObj",agentObj);
assignin("base","p",p);
assignin("base","par",par);
assignin("base","x0",x0);
assignin("base","x_prev0",x_prev0);
assignin("base","tau_prev0",tau_prev0);
assignin("base","Ts",Ts);
assignin("base","Tf",Tf);

mdl = "assumed_mode_ppo_simulink";
configure_ppo_simulink_model(mdl,false);
testScopes = find_system(mdl,LookUnderMasks="all", ...
    IncludeCommented="on",BlockType="Scope");
for k = 1:numel(testScopes)
    set_param(testScopes{k},"Commented","off");
end
set_param(mdl,"SimulationMode","normal","StopTime",num2str(Tf));
open_system(mdl);
configure_control_metric_logging(mdl,"MATLAB Function1");
simOut = sim(mdl,StopTime=num2str(Tf));
ppoMetrics = compute_control_metrics(simOut,par);
disp("Deterministic pure MEP-PPO test finished.");
