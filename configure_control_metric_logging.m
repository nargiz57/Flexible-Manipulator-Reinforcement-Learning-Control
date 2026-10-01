function configure_control_metric_logging(mdl,plantBlock)
%CONFIGURE_CONTROL_METRIC_LOGGING Log plant state and applied torque.

% The state is the output of the six-state plant integrator.  The applied
% torque is the second input of the plant MATLAB Function block.
set_param(mdl,"SignalLogging","on","SignalLoggingName","logsout");

integrator = mdl+"/Integrator";
statePorts = get_param(integrator,"PortHandles");
configureLine(statePorts.Outport(1),"state");

plantPorts = get_param(mdl+"/"+plantBlock,"PortHandles");
configureLine(plantPorts.Inport(2),"torque");
end

function configureLine(portHandle,signalName)
lineHandle = get_param(portHandle,"Line");
if lineHandle == -1
    error("MEP:MetricSignalNotConnected", ...
        "The %s metric signal is not connected.",signalName);
end
% Signal-logging properties live on the source output port.  When an input
% port is supplied (the applied-torque case), follow its line to the source.
if strcmp(get_param(portHandle,"PortType"),"inport")
    portHandle = get_param(lineHandle,"SrcPortHandle");
end
set_param(portHandle, ...
    "DataLogging","on", ...
    "DataLoggingNameMode","Custom", ...
    "DataLoggingName",signalName);
end
