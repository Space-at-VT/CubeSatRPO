% Solve for equations of motion
function [Aeqnew,beqnew] = setEOM(Aeqold,beqold,sat,R)
if nargin < 4 || isempty(R),R = sat.Rib;end  
scenario = sat.scenario;

dt = scenario.dt;
Nsim = scenario.Nsim;
Nvar = scenario.Nvar;
Ntotal = scenario.Ntotal;
alpha = sat.umax*dt/sat.m;  %position multiplier
beta = sat.umax*dt^2/sat.m; %velocity multiplier
t = sat.t(end);

% Add inequality contraints
Aeq = zeros(3*Nsim,Ntotal);
beq = zeros(3*Nsim,1);
for ii = 1:3:(3*Nsim) 
    nn = (ii+2)/3;
    
    % Define equtions of motion
    switch sat.EOM
        case 'HCW'
            A = HCW(scenario);
        case 'LERM'
            A = LERM(scenario,t);
    end
    C = A(4:6,:);
    
    jj = 1;
    for kk = 1:nn
        % Solve for state at each time step
        Aeq(ii:ii+2,jj:2:jj+5) = R*C(:,1:3)*beta*(nn-kk)+R*C(:,4:6)*alpha;
        Aeq(ii:ii+2,jj+1:2:jj+5) = -R*C(:,1:3)*beta*(nn-kk)-R*C(:,4:6)*alpha;
        
        % Set equivalence
        Aeq(ii:ii+2,Nvar+3*(nn-1)+1:Nvar+3*(nn-1)+3) = -eye(3);
        
        % Equations of motions
        if kk > 1            
            Aeq(ii:ii+2,Nvar+3*(kk-2)+1:Nvar+3*(kk-2)+3) = ...
                R*C(:,1:3)*(dt^2*(nn-kk+1))+R*C(:,4:6)*dt;
        end
        jj = jj+6;
    end 
    
    % Initial Conditions
    beq(ii:ii+2) = -C(:,1:3)*(sat.p+nn*dt*sat.v)'-C(:,4:6)*(sat.v)';
        
    % Update time
    t = t+dt;
end

% Update matrices
Aeqnew = [Aeqold;Aeq];
beqnew = [beqold;beq];
end