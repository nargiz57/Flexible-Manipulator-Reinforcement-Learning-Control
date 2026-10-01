function [alpha,gamma,gammaDot,E1,E2,E1Scale,E2Scale] = ...
    ppo_modal_quantities(x,par)
%PPO_MODAL_QUANTITIES Tip kinematics and retained modal energies.
%#codegen

phi1pL = par(9);
phi2pL = par(10);
Meta1 = par(11);
Meta2 = par(12);
Keta1 = par(13);
Keta2 = par(14);

alpha = phi1pL*x(3) + phi2pL*x(5);
gamma = x(1) + alpha;
gammaDot = x(2) + phi1pL*x(4) + phi2pL*x(6);

E1 = 0.5*Meta1*x(4)^2 + 0.5*Keta1*x(3)^2;
E2 = 0.5*Meta2*x(6)^2 + 0.5*Keta2*x(5)^2;

% For an undamped mode, kinetic and potential energies are equal at the
% selected displacement amplitude because M*wn^2 = K.
E1Scale = max(Keta1*(0.004)^2,1e-12);
E2Scale = max(Keta2*(0.001)^2,1e-12);
end
