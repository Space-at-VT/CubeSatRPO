function [Anew,bnew] = maxVelocity(Aold,bold,sat)
R = sat.Rib;
dt = sat.scenario.dt;
Nsim = sat.Nsim;
Nvar = sat.Nvar;
Ntotal = sat.Ntotal;
alpha = sat.umax*dt/sat.m;

A = zeros(6*Nsim,Ntotal);
b = zeros(6*Nsim,1);
for ii = 1:6:(6*Nsim) 
    jj = 1;           
    for kk = 1:(ii+5)/6     
        % Postive bounds    
        A(ii:ii+2,jj:2:jj+5) = R*alpha;
        A(ii:ii+2,jj+1:2:jj+5) = -R*alpha;
        % EOM
        if kk > 1
            A(ii:ii+2,Nvar+3*(kk-2)+1:Nvar+3*(kk-2)+3) = R*dt;
        end
        
        % Negative Bound
        A(ii+3:ii+5,:) = -A(ii:ii+2,:);
        jj = jj+6;
    end
    
    % Set equivalence 
    b(ii:ii+2) = (sat.vmax-sat.v)';
    b(ii+3:ii+5) = (sat.vmax+sat.v)';
end

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end