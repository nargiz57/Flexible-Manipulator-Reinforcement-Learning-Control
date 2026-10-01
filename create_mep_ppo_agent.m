function agent = create_mep_ppo_agent(obsInfo,actInfo,Ts,useGPU)
%CREATE_MEP_PPO_AGENT Create the direct-torque MEP-PPO actor and critic.
arguments
    obsInfo
    actInfo
    Ts (1,1) double {mustBePositive} = 0.02
    useGPU (1,1) logical = false
end

numObs = prod(obsInfo.Dimension);
numAct = prod(actInfo.Dimension);
tauMax = max(abs([actInfo.LowerLimit(:);actInfo.UpperLimit(:)]));

commonLayers = [
    featureInputLayer(numObs,Normalization="none",Name="observation")
    fullyConnectedLayer(128,WeightsInitializer="glorot",Name="actorFC1")
    tanhLayer(Name="actorTanh1")
    fullyConnectedLayer(128,WeightsInitializer="glorot",Name="actorFC2")
    tanhLayer(Name="actorTanh2")
];
meanLayers = [
    fullyConnectedLayer(numAct, ...
        WeightsInitializer="zeros",BiasInitializer="zeros",Name="meanFC")
    tanhLayer(Name="meanTanh")
    scalingLayer(Scale=tauMax,Name="mean")
];
stdFCLayer = fullyConnectedLayer(numAct, ...
    WeightsInitializer="zeros",BiasInitializer="zeros",Name="stdFC");
stdFCLayer.Bias = -2.0605*ones(numAct,1);
stdLayers = [
    stdFCLayer
    softplusLayer(Name="standardDeviation")
];

actorGraph = layerGraph(commonLayers);
actorGraph = addLayers(actorGraph,meanLayers);
actorGraph = addLayers(actorGraph,stdLayers);
actorGraph = connectLayers(actorGraph,"actorTanh2","meanFC");
actorGraph = connectLayers(actorGraph,"actorTanh2","stdFC");
actorNet = dlnetwork(actorGraph);

actor = rlContinuousGaussianActor( ...
    actorNet,obsInfo,actInfo, ...
    ObservationInputNames="observation", ...
    ActionMeanOutputNames="mean", ...
    ActionStandardDeviationOutputNames="standardDeviation");

criticNet = dlnetwork([
    featureInputLayer(numObs,Normalization="none",Name="observation")
    fullyConnectedLayer(128,WeightsInitializer="glorot",Name="criticFC1")
    tanhLayer(Name="criticTanh1")
    fullyConnectedLayer(128,WeightsInitializer="glorot",Name="criticFC2")
    tanhLayer(Name="criticTanh2")
    fullyConnectedLayer(1,WeightsInitializer="zeros",Name="value")
]);
critic = rlValueFunction(criticNet,obsInfo, ...
    ObservationInputNames="observation");

if useGPU
    actor.UseDevice = "gpu";
    critic.UseDevice = "gpu";
end

agentOpts = rlPPOAgentOptions;
agentOpts.SampleTime = Ts;
agentOpts.DiscountFactor = exp(-Ts/2.5);
agentOpts.ExperienceHorizon = 1200;
agentOpts.MiniBatchSize = 128;
agentOpts.NumEpoch = 4;
agentOpts.ClipFactor = 0.15;
agentOpts.EntropyLossWeight = 1e-4;
agentOpts.AdvantageEstimateMethod = "gae";
agentOpts.GAEFactor = 0.95;
agentOpts.NormalizedAdvantageMethod = "current";
agentOpts.ActorOptimizerOptions.LearnRate = 3e-4;
agentOpts.ActorOptimizerOptions.GradientThreshold = 1;
agentOpts.CriticOptimizerOptions.LearnRate = 5e-4;
agentOpts.CriticOptimizerOptions.GradientThreshold = 1;

agent = rlPPOAgent(actor,critic,agentOpts);
end
