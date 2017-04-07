function [Anew,bnew] = holdProximity(Aold,bold,sat,lbnd,ubnd)
R = sat.Rib;
dt = sat.scenario.dt;
Nsim = sat.Nsim;
Nvar = sat.Nvar;
Ntotal = sat.Ntotal;
beta = sat.umax*dt^2/sat.m; %velocity multiplier

% Add inequality contraints
A = zeros(6*Nsim,Ntotal);
b = zeros(6*Nsim,1);
for ii = 1:6:(6*Nsim); %counting position of rows (+/-x,y,z)
    jj = 1;     
    nn =(ii+5)/6;
    for kk = 1:nn
        A(ii:ii+2,jj:2:jj+5) = R*beta*(nn-kk);
        A(ii:ii+2,jj+1:2:jj+5) = -R*beta*(nn-kk);    
        
        % Equations of motion
        if kk > 1
            A(ii:ii+2,Nvar+3*(kk-2)+1:Nvar+3*(kk-2)+3) = R*dt^2*(nn-kk+1);
            A(ii+3:ii+5,Nvar+3*(kk-2)+1:Nvar+3*(kk-2)+3) = -R*dt^2*(nn-kk+1);
        end      
        jj = jj+6;
    end
    A(ii+3:ii+5,:) = -A(ii:ii+2,:);
    
    b(ii:ii+2) = (ubnd-sat.p)-nn*dt*sat.v;
    b(ii+3:ii+5) = -((lbnd-sat.p)-nn*dt*sat.v);    
end

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end