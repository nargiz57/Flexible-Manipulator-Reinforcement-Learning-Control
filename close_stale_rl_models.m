function close_stale_rl_models(activeModel)
%CLOSE_STALE_RL_MODELS Close clean inactive RL models left in memory.
arguments
    activeModel (1,1) string = "assumed_mode_ppo_simulink"
end

knownModels = [
    "assumed_mode_actor_critic_simulink"
    "assumed_mode_ddpg_simulink"
    "assumed_mode_ppo_simulink"
];

for k = 1:numel(knownModels)
    mdl = knownModels(k);
    if mdl == activeModel || ~bdIsLoaded(mdl)
        continue
    end
    if string(get_param(mdl,"Dirty")) == "on"
        warning("MEP:DirtyInactiveModel", ...
            "Inactive model %s has unsaved changes and was left open. "+ ...
            "It is not used by PPO, but Simulink may display a stale-file "+ ...
            "warning until you save or close it.",mdl);
        continue
    end
    close_system(mdl,0);
end
end
