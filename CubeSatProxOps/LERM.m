function A = LERM(scenario)
% Orginal Author: Andrew C. Rogers
t = scenario.t;
mu = scenario.mu;
a = scenario.a;
ecc = scenario.ecc;
nu = scenario.nu;

% Time varying coefficients and solution to Kepler's equation
n = sqrt(mu/a^3);
t0 = nu2t(n,nu,ecc);
M = n.*(t+t0);
[EccAnom,~] = kepler(M,ecc);
f = 2.*atan(sqrt((1+ecc)./(1-ecc)).*tan(EccAnom./2));
r = (a.*(1-ecc.^2))./(1+ecc.*cos(f));
fdot = sqrt(mu.*a.*(1-ecc.^2)).*(1+ecc.*cos(f)).^2./(a.^2.*(1-ecc.^2).^2);
rdot = ecc.*sin(f).*sqrt(mu.*a.*(1-ecc.^2))./(a.*(1-ecc.^2));
fddot = -2.*rdot.*fdot./r;

% State matrix for LERM
A = [0,0,0,1,0,0;
      0,0,0,0,1,0;
      0,0,0,0,0,1;
      (fdot^2+2*mu/r^3) fddot 0 0 2*fdot 0;
      -fddot (fdot^2-mu/r^3) 0 -2*fdot 0 0
      0 0 -mu/r^3 0 0 0];
end

function [E,iter] = kepler(M,ecc)
% Function solves Kepler's equation:
% M = n*(t-t_0) = E-e*sin(E)
% Using Newton-Raphson iteration
% AC Rogers, 21 February 2013
% Inputs:
%           M    = mean anomaly
%           e    = eccentricity
% Outputs:
%           E    = eccentric anomaly
%           iter = number of iterations
tol = 1e-12;
iter = 1;
for ii = 1:length(M)
    E_n = M(ii);
    f_n = E_n-ecc.*sin(E_n) - M(ii);
    while (abs(f_n) > tol)
        E_n = E_n - f_n./(1-ecc.*cos(E_n));
        f_n = E_n - ecc.*sin(E_n) - M(ii);
        iter = iter + 1;
    end
    E(ii) = E_n;
end
end

function t = nu2t(n,nu,ecc)
% Function solves for time from periapsis given mean motion, ecentricity,
% and true anomaly
% IE, 08/30/2016
E = acos((ecc+cosd(nu))/(1+ecc*cosd(nu)));
M = E-ecc*sin(E);
t = M/n;
end