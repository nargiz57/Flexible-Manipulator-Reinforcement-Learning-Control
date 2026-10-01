function par = build_ppo_parameter_vector(p,Ts,stage)
%BUILD_PPO_PARAMETER_VECTOR Pack plant and MEP-PPO configuration values.
arguments
    p (1,1) struct
    Ts (1,1) double {mustBePositive}
    stage (1,1) double {mustBeInteger,mustBeInRange(stage,1,3)} = 1
end

par = [
    p.L;p.l;p.J2;p.mr;p.mp;p.Btheta;
    p.phi1L;p.phi2L;p.phi1pL;p.phi2pL;
    p.Meta1;p.Meta2;p.Keta1;p.Keta2;p.Deta1;p.Deta2;
    p.Kp;p.Kd;p.theta_d;p.theta_dot_d;p.tau_max;
    p.rl.tau_max;
    p.rl.state_scale
];
par(32) = Ts;
par(33) = stage;
end
