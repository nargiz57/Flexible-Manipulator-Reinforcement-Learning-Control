function [bestAgent,bestMetrics,results] = ppo_select_best_checkpoint( ...
    checkpointDir,env,maxSteps,par,thetaD,fallbackAgent,stage)
% Prefer stage-passing, then lowest-score policy.
arguments
    checkpointDir (1,1) string
    env
    maxSteps (1,1) double {mustBeInteger,mustBePositive}
    par
    thetaD (1,1) double = pi/6
    fallbackAgent = []
    stage (1,1) double {mustBeInteger,mustBeInRange(stage,0,3)} = 0
end

files = dir(fullfile(checkpointDir,"Agent*.mat"));
resultTemplate = struct( ...
    "File","","Score",inf,"TotalReward",-inf, ...
    "PassedStage",false, ...
    "PassedNominalTracking",false);
results = repmat(resultTemplate,numel(files),1);
resultCount = 0;
bestAgent = fallbackAgent;
bestMetrics = struct("Score",inf);
bestPassedStage = false;

for k = 1:numel(files)
    candidateData = load(fullfile(files(k).folder,files(k).name), ...
        "saved_agent");
    candidate = candidateData.saved_agent(1);
    try
        candidateMetrics = ppo_evaluate_agent( ...
            candidate,env,maxSteps,par,thetaD);
    catch err
        warning("MEP:PPOCheckpointEvaluation", ...
            "Could not evaluate %s: %s",files(k).name,err.message);
        continue
    end

    resultCount = resultCount+1;
    candidatePassedStage = false;
    if stage > 0
        candidatePassedStage = ppo_stage_passed(candidateMetrics,stage);
    end
    results(resultCount) = struct( ...
        File=string(fullfile(files(k).folder,files(k).name)), ...
        Score=candidateMetrics.Score, ...
        TotalReward=candidateMetrics.TotalReward, ...
        PassedStage=candidatePassedStage, ...
        PassedNominalTracking=candidateMetrics.PassedNominalTracking);
    if isPreferred( ...
            candidatePassedStage,candidateMetrics.Score, ...
            bestPassedStage,bestMetrics.Score)
        bestAgent = candidate;
        bestMetrics = candidateMetrics;
        bestPassedStage = candidatePassedStage;
    end
end

% SaveAgentCriteria can omit the final in-memory update, so it must be
% evaluated as a candidate even when checkpoint files exist.
if ~isempty(fallbackAgent)
    fallbackMetrics = ppo_evaluate_agent( ...
        fallbackAgent,env,maxSteps,par,thetaD);
    fallbackPassedStage = false;
    if stage > 0
        fallbackPassedStage = ppo_stage_passed(fallbackMetrics,stage);
    end
    resultCount = resultCount+1;
    results(resultCount) = struct( ...
        File="<in-memory-final>", ...
        Score=fallbackMetrics.Score, ...
        TotalReward=fallbackMetrics.TotalReward, ...
        PassedStage=fallbackPassedStage, ...
        PassedNominalTracking=fallbackMetrics.PassedNominalTracking);
    if isPreferred( ...
            fallbackPassedStage,fallbackMetrics.Score, ...
            bestPassedStage,bestMetrics.Score)
        bestAgent = fallbackAgent;
        bestMetrics = fallbackMetrics;
    end
end
results = results(1:resultCount);
end

function preferred = isPreferred( ...
    candidatePassed,candidateScore,bestPassed,bestScore)
preferred = (candidatePassed && ~bestPassed) ...
    || (candidatePassed == bestPassed && candidateScore < bestScore);
end
