function [Aeqnew,beqnew] = setPhaseStateEq(Aeqold,beqold,sat,xf)
dt = sat.scenario.dt;
R = eye(3);
Nsim = sat.Nsim;
Nvar = sat.Nvar;
Ntotal = sat.Ntotal;
alpha = sat.umax*dt/sat.m;
beta = sat.umax*dt^2/sat.m; %Velocity multiplier

% Target objective absolute value
Aeq = zeros(6,Ntotal);
beq = zeros(6,1);

for ii = 1:Nsim 
    jj = 6*ii-5;
    % Positive bound
    Aeq(1:3,jj:2:jj+5) = R*beta*(Nsim-ii);
    Aeq(1:3,jj+1:2:jj+5) = -R*beta*(Nsim-ii);
    Aeq(4:6,jj:2:jj+5) = R*alpha;
    Aeq(4:6,jj+1:2:jj+5) = -R*alpha;
    
    % Equations of motion
    if ii > 1
        Aeq(1:3,Nvar+3*(ii-2)+1:Nvar+3*(ii-2)+3) = R*dt^2*(Nsim-ii+1);
        Aeq(4:6,Nvar+3*(ii-2)+1:Nvar+3*(ii-2)+3) = R*dt;
    end       
end

% Initial conditions
beq(1:3) = (xf(1:3)-sat.p-Nsim*dt*sat.v)';
beq(4:6) = (xf(4:6)-sat.v)';


% Update matrices
Aeqnew = [Aeqold;Aeq];
beqnew = [beqold;beq];
end