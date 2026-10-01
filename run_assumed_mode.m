clear; clc; close all;

%% Parameters

p.L = 1.0;
p.l = 0.3;

p.J2 = 0.005;
p.mr = 0.3;
p.mp = 0.2;
p.rho = 2700;
p.A = 1e-4;
p.E = 70e9;
p.I = 1e-10;

p.zeta = 0.02;
p.Btheta = 0.02;

%% First and Second cantilever modes

beta1L = 1.87510407;
beta2L = 4.69409113;

p.beta1 = beta1L / p.L;
p.beta2 = beta2L / p.L;

sigma1 = (cosh(beta1L) + cos(beta1L)) / ...
         (sinh(beta1L) + sin(beta1L));

sigma2 = (cosh(beta2L) + cos(beta2L)) / ...
         (sinh(beta2L) + sin(beta2L));

phi1 = @(x) cosh(p.beta1*x) - cos(p.beta1*x) ...
    - sigma1*(sinh(p.beta1*x) - sin(p.beta1*x));

phi2 = @(x) cosh(p.beta2*x) - cos(p.beta2*x) ...
    - sigma2*(sinh(p.beta2*x) - sin(p.beta2*x));

phi1_d = @(x) p.beta1*(sinh(p.beta1*x) + sin(p.beta1*x) ...
    - sigma1*(cosh(p.beta1*x) - cos(p.beta1*x)));

phi2_d = @(x) p.beta2*(sinh(p.beta2*x) + sin(p.beta2*x) ...
    - sigma2*(cosh(p.beta2*x) - cos(p.beta2*x)));

phi1_dd = @(x) p.beta1^2*(cosh(p.beta1*x) + cos(p.beta1*x) ...
    - sigma1*(sinh(p.beta1*x) + sin(p.beta1*x)));

phi2_dd = @(x) p.beta2^2*(cosh(p.beta2*x) + cos(p.beta2*x) ...
    - sigma2*(sinh(p.beta2*x) + sin(p.beta2*x)));

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

%% Controller parameters

p.theta_d = pi/6;
p.theta_dot_d = 0;

p.Kp = 1.1;
p.Kd = 0.95;

p.tau_max = 2.0;
% External controller / PS2F input limit
p.rl.tau_max = p.tau_max;

% For compatibility with RL/PS2F code
p.rl.state_scale = [ ...
    0.5;   % gamma error
    2.0;   % gamma_dot error
    0.5;   % theta error
    2.0;   % theta_dot error
    0.05;  % alpha
    0.5;   % alpha_dot
    0.05;  % eta1
    0.5;   % eta1_dot
    0.02;  % eta2
    0.5    % eta2_dot
];

%% Simulation setup

Ts = 0.02;
Tend = 10;

% x = [theta; theta_dot; eta1; eta1_dot; eta2; eta2_dot]
x0 = [0; 0; 0; 0; 0; 0];


%% Pack parameters into numeric vector for Simulink

par = [
    p.L;              % 1
    p.l;              % 2
    p.J2;             % 3
    p.mr;             % 4
    p.mp;             % 5
    p.Btheta;         % 6

    p.phi1L;          % 7
    p.phi2L;          % 8
    p.phi1pL;         % 9
    p.phi2pL;         % 10

    p.Meta1;          % 11
    p.Meta2;          % 12
    p.Keta1;          % 13
    p.Keta2;          % 14
    p.Deta1;          % 15
    p.Deta2;          % 16

    p.Kp;             % 17
    p.Kd;             % 18
    p.theta_d;        % 19
    p.theta_dot_d;    % 20
    p.tau_max;        % 21

    p.rl.tau_max;     % 22
    p.rl.state_scale  % 23 to 32
];
%% PS2F parameters

ps2f.Ts = Ts;

ps2f.N = 30;     % nominal MPC horizon
ps2f.M = 10;     % filter horizon, must be <= N
ps2f.a = 1.0;

ps2f.u_max = p.rl.tau_max;

% Nominal MPC terminal equality
ps2f.use_terminal_theta_eq = true;
ps2f.use_terminal_thetadot_eq = true;

% Filter OCP terminal equality
% Start with false for debugging. Turn true later if stable.
ps2f.use_filter_terminal_theta_eq = true;
ps2f.use_filter_terminal_thetadot_eq = false;

% Low-pass filter for theta reference
ps2f.use_ref_lpf = true;
ps2f.Tref = 0.5;     % seconds, try 0.3 to 0.8

% State constraints on shifted state:
% y = [theta - theta_d; theta_dot; eta1; eta1_dot; eta2; eta2_dot]
ps2f.lbX = [-1.00; -3.0; -0.08; -3.0; -0.05; -3.0];
ps2f.ubX = [ 1.00;  3.0;  0.08;  3.0;  0.05;  3.0];

% For first test, keep terminal set loose
ps2f.lbXf = ps2f.lbX;
ps2f.ubXf = ps2f.ubX;

% Angle-tracking MPC cost
ps2f.Qtheta    = 200;
ps2f.Qthetadot = 20;

% Small vibration penalties
ps2f.Qeta1     = 2;
ps2f.Qeta1dot  = 0.1;
ps2f.Qeta2     = 2;
ps2f.Qeta2dot  = 0.1;

% Input penalty
ps2f.R = 0.1;

% Terminal weights
ps2f.Ptheta    = 800;
ps2f.Pthetadot = 80;

ps2f.Peta1     = 2;
ps2f.Peta1dot  = 0.1;
ps2f.Peta2     = 2;
ps2f.Peta2dot  = 0.1;
%use_ps2f = 1;   % PID only
%assignin('base','use_ps2f',use_ps2f);
assignin('base','par',par);
assignin('base','x0',x0);
assignin('base','Ts',Ts);
assignin('base','Tend',Tend);
assignin('base','ps2f',ps2f);
%% Run Simulink model

model = "assumed_mode_wout_ps2f_simulink";
clear ps2f_exact_filter_mfile;
open_system(model);
configure_control_metric_logging(model,"MATLAB Function");
simOut = sim(model, "StopTime", num2str(Tend));
pidMetrics = compute_control_metrics(simOut,par);
