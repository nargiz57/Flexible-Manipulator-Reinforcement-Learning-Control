function p = build_assumed_mode_params_actor_critic()

%% Physical parameters

p.L = 1.0;          % flexible beam length [m]
p.l = 0.3;          % rigid second link length [m]

p.J2 = 0.005;       % rigid link equivalent inertia [kg m^2]
p.mr = 0.3;         % distributed rigid link mass [kg]
p.mp = 0.2;         % tip payload/tool mass [kg]

p.rho = 2700;       % density [kg/m^3]
p.A = 1e-4;         % cross-sectional area [m^2]
p.E = 70e9;         % Young's modulus [Pa]
p.I = 1e-10;        % second moment of area [m^4]

p.zeta = 0.02;      % modal damping ratio
p.Btheta = 0.02;    % joint damping [Nms/rad]

%% First and second cantilever modes

beta1L = 1.87510407;
beta2L = 4.69409113;

p.beta1 = beta1L / p.L;
p.beta2 = beta2L / p.L;

sigma1 = (cosh(beta1L) + cos(beta1L)) / ...
         (sinh(beta1L) + sin(beta1L));

sigma2 = (cosh(beta2L) + cos(beta2L)) / ...
         (sinh(beta2L) + sin(beta2L));

%% Mode shapes

phi1 = @(x) cosh(p.beta1*x) - cos(p.beta1*x) ...
    - sigma1*(sinh(p.beta1*x) - sin(p.beta1*x));

phi2 = @(x) cosh(p.beta2*x) - cos(p.beta2*x) ...
    - sigma2*(sinh(p.beta2*x) - sin(p.beta2*x));

%% First derivatives of mode shapes

phi1_d = @(x) p.beta1*(sinh(p.beta1*x) + sin(p.beta1*x) ...
    - sigma1*(cosh(p.beta1*x) - cos(p.beta1*x)));

phi2_d = @(x) p.beta2*(sinh(p.beta2*x) + sin(p.beta2*x) ...
    - sigma2*(cosh(p.beta2*x) - cos(p.beta2*x)));

%% Second derivatives of mode shapes

phi1_dd = @(x) p.beta1^2*(cosh(p.beta1*x) + cos(p.beta1*x) ...
    - sigma1*(sinh(p.beta1*x) + sin(p.beta1*x)));

phi2_dd = @(x) p.beta2^2*(cosh(p.beta2*x) + cos(p.beta2*x) ...
    - sigma2*(sinh(p.beta2*x) + sin(p.beta2*x)));

%% Tip values

p.phi1L = phi1(p.L);
p.phi2L = phi2(p.L);

p.phi1pL = phi1_d(p.L);
p.phi2pL = phi2_d(p.L);

%% Modal mass, stiffness, damping

p.Meta1 = integral(@(x) p.rho*p.A.*phi1(x).^2, 0, p.L);
p.Keta1 = integral(@(x) p.E*p.I.*phi1_dd(x).^2, 0, p.L);
p.wn1 = sqrt(p.Keta1 / p.Meta1);
p.Deta1 = 2*p.zeta*p.wn1*p.Meta1;

p.Meta2 = integral(@(x) p.rho*p.A.*phi2(x).^2, 0, p.L);
p.Keta2 = integral(@(x) p.E*p.I.*phi2_dd(x).^2, 0, p.L);
p.wn2 = sqrt(p.Keta2 / p.Meta2);
p.Deta2 = 2*p.zeta*p.wn2*p.Meta2;

%% Reference target

p.theta_d = pi/6;
p.theta_dot_d = 0;

%% PD gains kept only for compatibility

p.Kp = 1.1;
p.Kd = 0.95;

%% Torque limits

p.tau_max = 2.0;        % final plant torque saturation [Nm]

%% RL settings

p.rl.tau_max = p.tau_max;

% Observation normalisation scales
% obs = [
%   gamma_err;
%   gamma_dot_err;
%   theta_err;
%   theta_dot_err;
%   alpha;
%   alpha_dot;
%   eta1;
%   eta1_dot;
%   eta2;
%   eta2_dot
% ]

p.rl.state_scale = [ ...
    pi/3;     % theta tracking error
    2.00;     % theta rate
    0.020;    % eta1
    0.20;     % eta1 rate (documentation; block uses wn1*eta1 scale)
    0.005;    % eta2
    0.05;     % eta2 rate (documentation; block uses wn2*eta2 scale)
    1.50;     % previous torque
    pi/3;     % absolute theta
    pi/3;     % requested theta
    0.020];   % RL sample time (preserves the 32-element par layout)

%% Simulation settings

p.sim.T = 10;
p.sim.Ts = 0.02;

end
