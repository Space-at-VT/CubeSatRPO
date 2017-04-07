function [Anew,bnew] = setPhaseState(Aold,bold,sat,xf)
% Approach target cost function contraints
dt = sat.scenario.dt;
R = sat.Rib;

Nsim = sat.Nsim;
Nvar = sat.Nvar;
Neom = sat.Neom;
Nslack = sat.Nslack;
Ntotal = sat.Ntotal;
alpha = sat.umax*dt/sat.m;
beta = sat.umax*dt^2/sat.m; %Velocity multiplier

% Target objective absolute value
A = zeros(12,Ntotal);
b = zeros(12,1);

for ii = 1:Nsim 
    jj = 6*ii-5;
    % Positive bound
    A(1:3,jj:2:jj+5)    = R*beta*(Nsim-ii);
    A(1:3,jj+1:2:jj+5)  = -R*beta*(Nsim-ii);
    A(4:6,jj:2:jj+5)    = R*alpha;
    A(4:6,jj+1:2:jj+5)  = -R*alpha;
    
    % Equations of motion
    if ii > 1
        A(1:3,Nvar+3*(ii-2)+1:Nvar+3*(ii-2)+3) = R*dt^2*(Nsim-ii+1);
        A(4:6,Nvar+3*(ii-2)+1:Nvar+3*(ii-2)+3) = R*dt;
    end       
end
A(7:12,:) = -A(1:6,:);
A(1:12,Nvar+Neom+1:Nvar+Neom+Nslack) = -[eye(6);eye(6)];

% Initial conditions
b(1:3) = (xf(1:3)-sat.p-Nsim*dt*sat.v)';
b(4:6) = (xf(4:6)-sat.v)';
b(7:12) = -b(1:6);


% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end