function mdl = build_ppo_simulink_model(forceRebuild)
%BUILD_PPO_SIMULINK_MODEL Create a separate model for direct-torque PPO.
arguments
    forceRebuild (1,1) logical = false
end

sourceMdl = "assumed_mode_ddpg_simulink";
mdl = "assumed_mode_ppo_simulink";
targetFile = mdl+".slx";

if isfile(targetFile) && ~forceRebuild
    load_system(mdl);
    configure_ppo_simulink_model(mdl,false);
    return
end

if bdIsLoaded(mdl)
    close_system(mdl,0);
end
load_system(sourceMdl);
save_system(sourceMdl,mdl);
close_system(sourceMdl,0);
load_system(mdl);

% Disconnect the old five-input reward interface one branch at a time.
removeBranch(mdl,"Integrator/1","MATLAB Function2/1");
removeBranch(mdl,"RL Agent/1","MATLAB Function2/2");
removeBranch(mdl,"Unit Delay/1","MATLAB Function2/3");
removeBranch(mdl,"MATLAB Function3/1","MATLAB Function2/4");
removeBranch(mdl,"Constant2/1","MATLAB Function2/5");
removeBranch(mdl,"MATLAB Function/1","RL Agent/1");
removeBranch(mdl,"MATLAB Function2/1","RL Agent/2");
removeBranch(mdl,"MATLAB Function3/1","RL Agent/3");
removeBranch(mdl,"RL Agent/1","MATLAB Function1/2");
removeBranch(mdl,"RL Agent/1","Unit Delay/1");

% Replace the inherited DDPG agent block. A fresh library block avoids
% retaining cached nine-observation interface metadata in the mask.
agentPosition = get_param(mdl+"/RL Agent","Position");
delete_block(mdl+"/RL Agent");
load_system("rllib");
add_block("rllib/RL Agent",mdl+"/RL Agent", ...
    Position=agentPosition,Agent="agentObj");

% Updating the chart script creates the new x_prev reward input.
rt = sfroot;
setChartScript(rt,mdl+"/MATLAB Function", ...
    "function obs = observation_block(x,tau_prev,par)"+newline+ ...
    "%#codegen"+newline+ ...
    "obs = zeros(12,1);"+newline+ ...
    "obs = ppo_observation(x,tau_prev,par);"+newline+ ...
    "end");
setChartScript(rt,mdl+"/MATLAB Function2", ...
    "function reward = reward_block(x,x_prev,tau_applied,tau_applied_prev,done,par)"+newline+ ...
    "%#codegen"+newline+ ...
    "reward = 0.0;"+newline+ ...
    "reward = ppo_reward(x,x_prev,tau_applied,tau_applied_prev,done,par);"+newline+ ...
    "end");
setChartScript(rt,mdl+"/MATLAB Function3", ...
    "function done = done_block(x,par)"+newline+ ...
    "%#codegen"+newline+ ...
    "done = false;"+newline+ ...
    "done = ppo_done(x,par);"+newline+ ...
    "end");

add_block("simulink/Discrete/Unit Delay",mdl+"/Previous State", ...
    Position=[510 55 550 85], ...
    InitialCondition="x_prev0", ...
    SampleTime="Ts");
add_block("simulink/Discrete/Unit Delay", ...
    mdl+"/Previous Previous Torque", ...
    Position=[510 5 550 35], ...
    InitialCondition="tau_prev0", ...
    SampleTime="Ts");

add_line(mdl,"Integrator/1","Previous State/1","autorouting","on");
add_line(mdl,"Unit Delay/1","Previous Previous Torque/1", ...
    "autorouting","on");
add_line(mdl,"Integrator/1","MATLAB Function2/1","autorouting","on");
add_line(mdl,"Previous State/1","MATLAB Function2/2","autorouting","on");
add_line(mdl,"Unit Delay/1","MATLAB Function2/3","autorouting","on");
add_line(mdl,"Previous Previous Torque/1","MATLAB Function2/4", ...
    "autorouting","on");
add_line(mdl,"MATLAB Function3/1","MATLAB Function2/5","autorouting","on");
add_line(mdl,"Constant2/1","MATLAB Function2/6","autorouting","on");
add_line(mdl,"MATLAB Function/1","RL Agent/1","autorouting","on");
add_line(mdl,"MATLAB Function2/1","RL Agent/2","autorouting","on");
add_line(mdl,"MATLAB Function3/1","RL Agent/3","autorouting","on");
add_line(mdl,"RL Agent/1","MATLAB Function1/2","autorouting","on");
add_line(mdl,"RL Agent/1","Unit Delay/1","autorouting","on");

set_param(mdl+"/Unit Delay", ...
    InitialCondition="tau_prev0",SampleTime="Ts");
set_param(mdl, ...
    StopTime="6", ...
    SolverType="Fixed-step", ...
    Solver="ode4", ...
    FixedStep="0.002", ...
    SimulationMode="normal");

save_system(mdl,[],"OverwriteIfChangedOnDisk",true);
end

function removeBranch(mdl,source,destination)
try
    delete_line(mdl,source,destination);
catch err
    if ~contains(err.message,"does not exist","IgnoreCase",true)
        rethrow(err);
    end
end
end

function setChartScript(rt,path,script)
chart = find(rt,"-isa","Stateflow.EMChart","Path",path);
assert(isscalar(chart),"Expected one MATLAB Function chart at %s.",path);
chart.Script = char(script);
end
