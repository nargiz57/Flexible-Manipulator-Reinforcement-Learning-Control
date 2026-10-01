clear; clc; close all;
clear ps2f_exact_filter_mfile1;

%% Load the PPO policy
% Prefer the published controller, but allow the stage-2 agent currently in
% this project to be tested without renaming it.
if isfile("trained_mep_ppo.mat")
    agentFile = "trained_mep_ppo.mat";
elseif isfile("mep_ppo_stage2_agent.mat")
    agentFile = "mep_ppo_stage2_agent.mat";
else
    error("MEP:NoPPOAgent", ...
        "No trained_mep_ppo.mat or mep_ppo_stage2_agent.mat was found.");
end

S = load(agentFile);
if isfield(S,"agent")
    agentObj = S.agent;
elseif isfield(S,"agentObj")
    agentObj = S.agentObj;
else
    error("MEP:NoPPOAgentVariable", ...
        "No agent or agentObj variable was found in %s.",agentFile);
end
agentObj.UseExplorationPolicy = false;

if isfield(S,"p")
    p = S.p;
else
    p = build_assumed_mode_params_actor_critic();
end
if isfield(S,"Ts")
    Ts = S.Ts;
else
    Ts = p.sim.Ts;
end

Tf = 10;
x0 = [0; 0; 0; 0; 0; 0];
x_prev0 = x0;
tau_prev0 = 0;
par = build_ppo_parameter_vector(p,Ts,2);
%par(5) = 1.30*par(5);       % payload +30%
%par(13:14) = 0.75*par(13:14); % modal stiffness -25%
%par(15:16) = 0.70*par(15:16); % modal damping -30%
%% Exact PS2F configuration
ps2f.Ts = 0.02;
ps2f.N = 50;
ps2f.M = 15;
ps2f.a = 0.98;
ps2f.u_max = p.tau_max;

ps2f.use_terminal_full_eq = true;
ps2f.use_terminal_theta_eq = true;
ps2f.use_terminal_thetadot_eq = true;
ps2f.use_filter_terminal_all_eq = true;
ps2f.use_filter_terminal_theta_eq = true;
ps2f.use_filter_terminal_thetadot_eq = false;

ps2f.use_ref_lpf = false;
ps2f.Tref = 0.5;
ps2f.lbX = [-0.60; -3.0; -0.08; -3.0; -0.05; -3.0];
ps2f.ubX = [ 0.02;  3.0;  0.08;  3.0;  0.05;  3.0];

ps2f.lbXf = ps2f.lbX;
ps2f.ubXf = ps2f.ubX;

ps2f.Qtheta = 200;    ps2f.Qthetadot = 20;
ps2f.Qeta1 = 2;       ps2f.Qeta1dot = 0.1;
ps2f.Qeta2 = 2;       ps2f.Qeta2dot = 0.1;
ps2f.R = 0.1;
ps2f.Ptheta = 800;    ps2f.Pthetadot = 80;
ps2f.Peta1 = 2;       ps2f.Peta1dot = 0.1;
ps2f.Peta2 = 2;       ps2f.Peta2dot = 0.1;
ps2f.scl = struct("s1",0.08,"s2",0.05);

% false enables the actual PPO-to-PS2F filtering stage.  Set true only to
% diagnose the nominal MPC independently of the PPO policy.
ps2f.debug_nominal_only = false;

%% Publish variables and simulate
assignin("base","agentObj",agentObj);
assignin("base","p",p);
assignin("base","par",reshape(par,[],1));
assignin("base","x0",x0);
assignin("base","x_prev0",x_prev0);
assignin("base","tau_prev0",tau_prev0);
assignin("base","Ts",Ts);
assignin("base","Tf",Tf);
assignin("base","ps2f",ps2f);

mdl = build_ppo_ps2f_simulink_model(false);
set_param(mdl,"SimulationMode","normal","StopTime",num2str(Tf));
open_system(mdl);
%configure_control_metric_logging(mdl,"MATLAB Function1");
clear ps2f_exact_filter_mfile1;
simOut = sim(mdl,StopTime=num2str(Tf));
%ppoPs2fMetrics = compute_control_metrics(simOut,par);

fprintf("Deterministic PPO + exact PS2F test finished using %s.\n",agentFile);
