function [Anew,bnew] = minDistancetest(Aold,bold,sat,scenario,pf)
% Approach target cost function contraints
dt = scenario.dt;
R = sat.Rib;

Nsim = scenario.Nsim;
Nvar = scenario.Nvar;
Neom = scenario.Neom;
Ntotal = scenario.Ntotal;
beta = sat.umax*dt^2/sat.m; %Velocity multiplier

% Target objective absolute value
A = zeros(6,Ntotal);
b = zeros(6,1);
      

for ii = 1:Nsim 
    jj = 6*ii-5;
    % Positive bound
    A(1:3,jj:2:jj+5) = R*beta*(Nsim-ii);
    A(1:3,jj+1:2:jj+5) = -R*beta*(Nsim-ii);
    
    % Equations of motion
    if ii > 1
        A(1:3,Nvar+3*(ii-2)+1:Nvar+3*(ii-2)+3) = R*dt^2*(Nsim-ii+1);
    end       
end
% Negative bound
A(4:6,:) = -A(1:3,:);

% Set equivalence
A(1:6,Nvar+Neom+1:Nvar+Neom+3) = -[eye(3);eye(3)];

% Initial conditions
b(1:3) = (-sat.p+pf-Nsim*dt*sat.v)';
b(4:6) = -(-sat.p+pf-Nsim*dt*sat.v)';

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end