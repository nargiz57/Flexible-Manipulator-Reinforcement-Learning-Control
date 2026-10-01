function tau_safe = ps2f_exact_filter_mfile1(x, tau_ext, par)
% PS2F exact filter with optional LQR terminal cost/set, running the
% prediction rollout through the SCALED-coordinate plant model
%
% State convention:
%   x = [theta; theta_dot; eta1; eta1_dot; eta2; eta2_dot]   (PHYSICAL)
% Shifted state:
%   y = x - xref,  xref = [theta_ref;0;0;0;0;0]
%
% scl = ps2f.scl (struct with fields s1, s2) selects the modal coordinate
% scaling used ONLY inside rk4_step_x's integration - everything else
% (costs, constraints, bounds) stays in physical coordinates.
%
% Important: if ps2f.use_lqr_terminal = true, the nominal MPC terminal
% condition is an ellipsoidal set y'P_lqr*y <= gamma_Xf and NOT a hard
% equality y_N = 0.
% This implementation was adapted from the method presented in:
% Y. Yan, C. Tao, J. Su, C. Liu, and S. Li, "MPC as a copilot: A predictive filter
% framework with safety and stability guarantees," 2026.

persistent mem

if isempty(mem)
    mem.vNom = [];
    mem.uFilt = [];
    mem.nomFailCount = 0;
    mem.filtFailCount = 0;
end

if ~isfield(mem,"nomFailCount")
    mem.nomFailCount = 0;
end
if ~isfield(mem,"filtFailCount")
    mem.filtFailCount = 0;
end

ps2f = evalin("base","ps2f");

if ~isfield(ps2f,"scl")
    error("ps2f.scl is not set. Add ps2f.scl = struct('s1',0.08,'s2',0.05); (or your chosen values) before assigning ps2f to the base workspace.");
end
scl = ps2f.scl;

N = ps2f.N;
M = ps2f.M;

if M > N
    error("PS2F requires M <= N.");
end

uMax = ps2f.u_max;
tau_ext = max(min(tau_ext, uMax), -uMax);

% ------------------------------------------------------------
% Reference handling
% ------------------------------------------------------------
theta_goal = par(19);   % final desired theta

if ~isfield(mem,"theta_cmd") || isempty(mem.theta_cmd) || ~isfinite(mem.theta_cmd)
    mem.theta_cmd = x(1);   % start from current theta
end

if isfield(ps2f,"use_ref_lpf") && ps2f.use_ref_lpf
    Tref = ps2f.Tref;
    alpha_ref = ps2f.Ts / (Tref + ps2f.Ts);
else
    alpha_ref = 1.0;        % no filtering
end

mem.theta_cmd = mem.theta_cmd + alpha_ref*(theta_goal - mem.theta_cmd);
theta_ref = mem.theta_cmd;

xref = get_xref(theta_ref);
y0 = x - xref;

%% fmincon options
opts = optimoptions("fmincon", ...
    "Algorithm","sqp", ...
    "Display","none", ...
    "MaxIterations",80, ...
    "MaxFunctionEvaluations",5000, ...
    "ConstraintTolerance",1e-5, ...
    "OptimalityTolerance",1e-5, ...
    "StepTolerance",1e-6);

%% Step 1: Nominal MPC PN(x)
if isempty(mem.vNom) || length(mem.vNom) ~= N
    ueq = get_u_eq(ps2f);
    v0 = ueq*ones(N,1);
else
    v0 = [mem.vNom(2:end); mem.vNom(end)];
end

lbU_N = -uMax*ones(N,1);
ubU_N =  uMax*ones(N,1);

[vStar, ~, exitNom] = fmincon( ...
    @(v) nominal_cost(v, y0, par, ps2f, xref, scl), ...
    v0, [], [], [], [], lbU_N, ubU_N, ...
    @(v) nominal_constraints(v, y0, par, ps2f, xref, scl), ...
    opts);

if exitNom <= 0
    tau_safe = emergency_fallback(x, theta_ref, uMax);

    mem.nomFailCount = mem.nomFailCount + 1;

    theta_err = x(1) - theta_ref;

    fprintf("[PS2F WARNING] Emergency fallback used. Count = %d, exitNom = %d, theta = %.4f, theta_err = %.4f, tau_RL = %.4f, tau_safe = %.4f\n", ...
        mem.nomFailCount, exitNom, x(1), theta_err, tau_ext, tau_safe);

    return;
end

[zStar, ~] = rollout_shifted(y0, vStar, par, ps2f.Ts, xref, scl);
mem.vNom = vStar;

% Debug mode: use nominal MPC directly.
if isfield(ps2f,"debug_nominal_only") && ps2f.debug_nominal_only
    tau_safe = max(min(vStar(1), uMax), -uMax);
    return;
end

%% Step 2: PS2F filtering OCP Pf,M(x)
if isempty(mem.uFilt) || length(mem.uFilt) ~= M
    u0 = tau_ext*ones(M,1);
else
    u0 = [mem.uFilt(2:end); mem.uFilt(end)];
    u0(1) = tau_ext;
end

lbU_M = -uMax*ones(M,1);
ubU_M =  uMax*ones(M,1);

[uStar, ~, exitFilt] = fmincon( ...
    @(u) filter_cost(u, tau_ext, vStar, ps2f), ...
    u0, [], [], [], [], lbU_M, ubU_M, ...
    @(u) filter_constraints(u, y0, zStar, vStar, par, ps2f, xref, scl), ...
    opts);

if exitFilt <= 0
    tau_safe = vStar(1);

    mem.filtFailCount = mem.filtFailCount + 1;

    fprintf("[PS2F NOTICE] Filter OCP failed. Using nominal input. Count = %d, exitFilt = %d, tau_RL = %.4f, tau_nom = %.4f\n", ...
        mem.filtFailCount, exitFilt, tau_ext, vStar(1));
else
    tau_safe = uStar(1);
    mem.uFilt = uStar;
end

tau_safe = max(min(tau_safe, uMax), -uMax);

end

function tau_out = ps2f_wrapper(x, tau_ext, par, use_ps2f)
%#codegen

if use_ps2f == 0
    tau_out = tau_ext;
    return;
end

tau_out = ps2f_exact_filter_mfile1(x, tau_ext, par);

end

function J = nominal_cost(v, y0, par, ps2f, xref, scl)

[ySeq, ~] = rollout_shifted(y0, v, par, ps2f.Ts, xref, scl);

N = length(v);
J = 0;

for i = 1:N
    y = ySeq(:,i);
    u = v(i);
    J = J + stage_cost(y, u, ps2f);
end

yN = ySeq(:,N+1);
J = J + terminal_cost(yN, ps2f);

end

function [c, ceq] = nominal_constraints(v, y0, par, ps2f, xref, scl)

[ySeq, ~] = rollout_shifted(y0, v, par, ps2f.Ts, xref, scl);

N = length(v);

c = [];

for i = 1:N
    y = ySeq(:,i);
    c = [c;
         y - ps2f.ubX;
         ps2f.lbX - y];
end

yN = ySeq(:,N+1);

if isfield(ps2f,"use_lqr_terminal") && ps2f.use_lqr_terminal
    % LQR terminal set:
    %   X_f = { y : y' P_lqr y <= gamma_Xf }
    % No equality constraint is used. The terminal state only needs to land
    % inside a local invariant region where the local LQR can handle the rest.
    c = [c;
         yN' * ps2f.P_lqr * yN - ps2f.gamma_Xf];
    ceq = [];
elseif isfield(ps2f,"use_terminal_full_eq") && ps2f.use_terminal_full_eq
    % Full-state terminal equality: Xf = {xref}, y(N) = 0 exactly.
    ceq = yN;
else
    % Terminal set.
    c = [c;
         yN - ps2f.ubXf;
         ps2f.lbXf - yN];

    if isfield(ps2f,"use_terminal_theta_eq") && ps2f.use_terminal_theta_eq
        ceq = yN(1);          % theta error must be zero

        if isfield(ps2f,"use_terminal_thetadot_eq") && ps2f.use_terminal_thetadot_eq
            ceq = [ceq; yN(2)];   % theta_dot must also be zero
        end
    else
        ceq = [];
    end
end

end

function J = filter_cost(u, tau_ext, vStar, ps2f)

J = (u(1) - tau_ext)^2;

if isfield(ps2f,"lambda_nom") && ps2f.lambda_nom > 0
    J = J + ps2f.lambda_nom*(u(1) - vStar(1))^2;
end

if isfield(ps2f,"lambda_seq") && ps2f.lambda_seq > 0
    m = min(length(u), length(vStar));
    J = J + ps2f.lambda_seq*sum((u(1:m) - vStar(1:m)).^2);
end

end

function [c, ceq] = filter_constraints(u, y0, zStar, vStar, par, ps2f, xref, scl)

M = length(u);

[xSeq, ~] = rollout_shifted(y0, u, par, ps2f.Ts, xref, scl);

c = [];

for i = 1:M
    y = xSeq(:,i);
    c = [c;
         y - ps2f.ubX;
         ps2f.lbX - y];
end

L_filter = 0;
L_nom = 0;

for i = 1:M
    L_filter = L_filter + stage_cost(xSeq(:,i), u(i), ps2f);
    L_nom    = L_nom    + stage_cost(zStar(:,i), vStar(i), ps2f);
end

ell0 = stage_cost(y0, u(1), ps2f);
c_perf = L_filter - L_nom - ps2f.a*ell0;
c = [c; c_perf];

ceq = [];

if isfield(ps2f,"use_filter_terminal_all_eq") && ps2f.use_filter_terminal_all_eq
    ceq = xSeq(:,M+1) - zStar(:,M+1);
else
    if isfield(ps2f,"use_filter_terminal_theta_eq") && ps2f.use_filter_terminal_theta_eq
        ceq = [ceq;
               xSeq(1,M+1) - zStar(1,M+1)];
    end

    if isfield(ps2f,"use_filter_terminal_thetadot_eq") && ps2f.use_filter_terminal_thetadot_eq
        ceq = [ceq;
               xSeq(2,M+1) - zStar(2,M+1)];
    end
end

end

function [ySeq, xSeq] = rollout_shifted(y0, uSeq, par, Ts, xref, scl)

nx = 6;
N = length(uSeq);

ySeq = zeros(nx,N+1);
xSeq = zeros(nx,N+1);

ySeq(:,1) = y0;
xSeq(:,1) = y0 + xref;

for i = 1:N
    xNow = xSeq(:,i);
    uNow = uSeq(i);

    xNext = rk4_step_x(xNow, uNow, par, Ts, scl);

    xSeq(:,i+1) = xNext;
    ySeq(:,i+1) = xNext - xref;
end

end

function xNext = rk4_step_x(x, u, par, Ts, scl)
% x is PHYSICAL. Convert to scaled coordinates only for the integration
% step (so the optimizer's internal rollout uses the better-conditioned
% mass matrix), then convert the result back to physical coordinates.
% Everything outside this function (costs, constraints, bounds, xref)
% stays in physical units, unchanged from the original file.

x_s = scaled_coord_convert(x, scl, "to_scaled");

k1 = assumed_mode_rhs_scaled_mfile(x_s,             u, par, scl);
k2 = assumed_mode_rhs_scaled_mfile(x_s + 0.5*Ts*k1, u, par, scl);
k3 = assumed_mode_rhs_scaled_mfile(x_s + 0.5*Ts*k2, u, par, scl);
k4 = assumed_mode_rhs_scaled_mfile(x_s + Ts*k3,     u, par, scl);

x_s_next = x_s + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);

xNext = scaled_coord_convert(x_s_next, scl, "to_physical");

end

function J = stage_cost(y, u, ps2f)

theta_err = y(1);
theta_dot = y(2);
eta1      = y(3);
eta1_dot  = y(4);
eta2      = y(5);
eta2_dot  = y(6);

u_eq = get_u_eq(ps2f);
u_dev = u - u_eq;

J = ps2f.Qtheta    * theta_err^2 + ...
    ps2f.Qthetadot * theta_dot^2 + ...
    ps2f.R         * u_dev^2;

J = J + ...
    ps2f.Qeta1     * eta1^2 + ...
    ps2f.Qeta1dot  * eta1_dot^2 + ...
    ps2f.Qeta2     * eta2^2 + ...
    ps2f.Qeta2dot  * eta2_dot^2;

end

function J = terminal_cost(y, ps2f)

if isfield(ps2f,"use_lqr_terminal") && ps2f.use_lqr_terminal
    J = y' * ps2f.P_lqr * y;
    return;
end

theta_err = y(1);
theta_dot = y(2);
eta1      = y(3);
eta1_dot  = y(4);
eta2      = y(5);
eta2_dot  = y(6);

J = ps2f.Ptheta    * theta_err^2 + ...
    ps2f.Pthetadot * theta_dot^2;

J = J + ...
    ps2f.Peta1     * eta1^2 + ...
    ps2f.Peta1dot  * eta1_dot^2 + ...
    ps2f.Peta2     * eta2^2 + ...
    ps2f.Peta2dot  * eta2_dot^2;

end

function xref = get_xref(theta_ref)

xref = [theta_ref;
        0;
        0;
        0;
        0;
        0];

end

function ueq = get_u_eq(ps2f)

if isfield(ps2f,"u_eq") && isfinite(ps2f.u_eq)
    ueq = ps2f.u_eq;
else
    ueq = 0;
end

end

function tau = emergency_fallback(x, theta_ref, uMax)

theta     = x(1);
theta_dot = x(2);

theta_err = theta - theta_ref;

tau = -0.4*theta_err - 0.2*theta_dot;

tau = max(min(tau, uMax), -uMax);

end
