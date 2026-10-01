function obs = ppo_observation(x,tauPrev,par)
%#codegen

thetaD = par(19);
tauMax = par(22);
wn1 = sqrt(par(13)/par(11));
wn2 = sqrt(par(14)/par(12));
[alpha,gamma,gammaDot,E1,E2,E1Scale,E2Scale] = ...
    ppo_modal_quantities(x,par);

obs = zeros(12,1);
obs(1) = (gamma-thetaD)/(pi/6);
obs(2) = gammaDot/1.0;
obs(3) = (x(1)-thetaD)/(pi/6);
obs(4) = x(2)/1.0;
obs(5) = x(3)/0.010;
obs(6) = x(4)/(wn1*0.010);
obs(7) = x(5)/0.0025;
obs(8) = x(6)/(wn2*0.0025);
obs(9) = alpha/(3*pi/180);
obs(10) = E1/E1Scale;
obs(11) = E2/E2Scale;
obs(12) = tauPrev/tauMax;

for k = 1:12
    obs(k) = min(max(obs(k),-5),5);
end
end
