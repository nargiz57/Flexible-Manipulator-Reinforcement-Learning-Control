function dx_s = assumed_mode_rhs_scaled_mfile(x_s, tau, par, scl)
% ASSUMED_MODE_RHS_SCALED_MFILE
%   Physical coords : q  = [theta;  eta1;      eta2]
%   Scaled coords    : q_s = [theta; eta1/s1;  eta2/s2]
%   i.e.  eta1 = s1*eta1_s ,  eta2 = s2*eta2_s
%
%   This is a pure diagonal change of generalized coordinates, q = S*q_s
%   with S = diag(1, s1, s2). It does not change the dynamics. What
%   it changes is the conditioning of the mass matrix used inside the
%   optimizer at every rollout step:
%
%       cond(M_raw)    up to ~1740  over a representative operating envelope
%       cond(M_scaled) up to ~60    over the SAME envelope  (~30x better)
%
%   because the raw mass matrix mixes the hub inertia J2 (~0.005, a
%   design choice) with the beam's modal mass Meta1/Meta2 (~0.27, set by
%   material/geometry) directly in absolute units, which are simply
%   different physical scales, not comparable numbers. Rescaling the
%   modal coordinates by their expected operating amplitude removes that mismatch.
%
%   Inputs
%     x_s  : scaled state [6x1] = [theta; theta_dot; eta1_s; eta1_s_dot; eta2_s; eta2_s_dot]
%     tau  : applied torque [scalar]
%     par  : SAME parameter vector as assumed_mode_rhs_mfile.m [32x1]
%     scl  : struct with fields s1, s2 (modal coordinate scale factors)
%   Output
%     dx_s : scaled state derivative [6x1]

dx_s = zeros(6,1);

%% Unpack parameters

l       = par(2);
J2      = par(3);
mr      = par(4);
mp      = par(5);
Btheta  = par(6);

phi1L   = par(7);
phi2L   = par(8);
phi1pL  = par(9);
phi2pL  = par(10);

Meta1   = par(11);
Meta2   = par(12);
Keta1   = par(13);
Keta2   = par(14);
Deta1   = par(15);
Deta2   = par(16);

tau_max = par(21);

s1 = scl.s1;
s2 = scl.s2;

%% States (scaled)

theta      = x_s(1);
theta_dot  = x_s(2);

eta1_s     = x_s(3);
eta1_s_dot = x_s(4);

eta2_s     = x_s(5);
eta2_s_dot = x_s(6);

qs    = [theta;      eta1_s;     eta2_s];
qsdot = [theta_dot;  eta1_s_dot; eta2_s_dot];

%% Saturate torque

tau = max(min(tau, tau_max), -tau_max);

%% Scaled geometry vectors  (gvec_s = S*gvec, wvec_s = S*wvec, S = diag(1,s1,s2))

gvec_s = [1; phi1pL*s1; phi2pL*s2];
wvec_s = [0; phi1L*s1;  phi2L*s2];

gamma = gvec_s.' * qs;

%% Rigid link and payload coefficients
Ir = mr*l^2/3;
Ip = mp*l^2;

Arot   = Ir + Ip;
Mvert  = mr + mp;
Across = mr*l/2 + mp*l;

%% Scaled mass matrix
% M_s(q_s) = S' * M(q) * S  , with S = diag(1,s1,s2).

Ms = zeros(3,3);

Ms(1,1) = Ms(1,1) + J2;
Ms(2,2) = Ms(2,2) + Meta1*s1^2;
Ms(3,3) = Ms(3,3) + Meta2*s2^2;

Ms = Ms ...
    + Arot*(gvec_s*gvec_s') ...
    + Mvert*(wvec_s*wvec_s') ...
    + Across*cos(gamma)*(gvec_s*wvec_s' + wvec_s*gvec_s');

%% Nonlinear velocity terms

dM = zeros(3,3,3);

common_dM_dgamma = -Across*sin(gamma)*(gvec_s*wvec_s' + wvec_s*gvec_s');

for kk = 1:3
    dM(:,:,kk) = common_dM_dgamma * gvec_s(kk);
end

h = zeros(3,1);

for ii = 1:3
    for jj = 1:3
        for kk = 1:3
            Gamma_ijk = 0.5*(dM(ii,jj,kk) + dM(ii,kk,jj) - dM(jj,kk,ii));
            h(ii) = h(ii) + Gamma_ijk*qsdot(jj)*qsdot(kk);
        end
    end
end

%% Stiffness and damping (congruence-scaled: K_s = S*K*S, D_s = S*D*S)

Kq_s = [0;
        Keta1*s1^2*eta1_s;
        Keta2*s2^2*eta2_s];

Dq_s = [Btheta*theta_dot;
        Deta1*s1^2*eta1_s_dot;
        Deta2*s2^2*eta2_s_dot];

Q_s = [tau; 0; 0];

%% Solve dynamics

qdd_s = Ms \ (Q_s - h - Dq_s - Kq_s);

%% Output derivative

dx_s(1) = theta_dot;
dx_s(2) = qdd_s(1);

dx_s(3) = eta1_s_dot;
dx_s(4) = qdd_s(2);

dx_s(5) = eta2_s_dot;
dx_s(6) = qdd_s(3);

end
