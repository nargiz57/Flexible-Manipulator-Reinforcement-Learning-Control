function mdl = build_ppo_ps2f_simulink_model(forceRebuild)
%BUILD_PPO_PS2F_SIMULINK_MODEL Add the exact PS2F to the direct-torque PPO model.
arguments
    forceRebuild (1,1) logical = false
end

sourceMdl = "assumed_mode_ppo_simulink";
mdl = "assumed_mode_ppo_ps2f_simulink";

if isfile(mdl+".slx") && ~forceRebuild
    load_system(mdl);
    return
end

if bdIsLoaded(mdl)
    close_system(mdl,0);
end

build_ppo_simulink_model(false);
load_system(sourceMdl);
save_system(sourceMdl,mdl);
close_system(sourceMdl,0);
load_system(mdl);

delete_line_if_present(mdl,"RL Agent/1","MATLAB Function1/2");
delete_line_if_present(mdl,"RL Agent/1","Unit Delay/1");

add_block("simulink/User-Defined Functions/MATLAB Function", ...
    mdl+"/PS2F Exact Filter",Position=[505 155 625 225]);
rt = sfroot;
chart = find(rt,"-isa","Stateflow.EMChart", ...
    "Path",mdl+"/PS2F Exact Filter");
assert(isscalar(chart),"Could not create the PS2F MATLAB Function block.");
chart.Script = sprintf([ ...
    'function tau_safe = ps2f_filter_block(x,tau_ppo,par)\n' ...
    '%%#codegen\n' ...
    'tau_safe = 0.0;\n' ...
    'coder.extrinsic(''ps2f_exact_filter_mfile1'');\n' ...
    'tau_safe = ps2f_exact_filter_mfile1(x,tau_ppo,par);\n' ...
    'end\n']);

add_block("simulink/Sources/Constant",mdl+"/PS2F Parameters", ...
    Value="par",Position=[505 245 535 275]);
add_line(mdl,"Integrator/1","PS2F Exact Filter/1","autorouting","on");
add_line(mdl,"RL Agent/1","PS2F Exact Filter/2","autorouting","on");
add_line(mdl,"PS2F Parameters/1","PS2F Exact Filter/3","autorouting","on");
add_line(mdl,"PS2F Exact Filter/1","MATLAB Function1/2","autorouting","on");
add_line(mdl,"PS2F Exact Filter/1","Unit Delay/1","autorouting","on");

save_system(mdl,[],"OverwriteIfChangedOnDisk",true);
end

function delete_line_if_present(mdl,src,dst)
try
    delete_line(mdl,src,dst);
catch err
    if ~contains(err.message,"does not exist","IgnoreCase",true)
        rethrow(err)
    end
end
end
