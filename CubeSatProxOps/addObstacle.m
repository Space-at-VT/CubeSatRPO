function [Anew,bnew] = addObstacle(Aold,bold,sat,lbnd,ubnd,N)
R = sat.Rib;

dt = sat.scenario.dt;
Nsim = sat.Nsim;
Nvar = sat.Nvar;
Neom = sat.Neom;
Ntotal = sat.Ntotal;
Nslack = sat.Nslack;
beta = sat.umax*dt^2/sat.m; %velocity multiplier
M = 1e6;

% Safety buffer
lbnd = lbnd-sat.dsafe;
ubnd = ubnd+sat.dsafe;

% Add inequality contraints
A = zeros(Nsim*7,Ntotal);
b = zeros(Nsim*7,1);
for ii = 1:7:(7*Nsim) %counting position of rows (+/-x,y,z...binary)
    jj = 1;           %counting position of columns (in u())
    
    nn = (ii+6)/7;
    for kk = 1:nn      %counting no. iterations
        A(ii:ii+2,jj:2:jj+5) = R*beta*(nn-kk);
        A(ii:ii+2,jj+1:2:jj+5) = -R*beta*(nn-kk);
        if kk > 1
            A(ii:ii+2,Nvar+3*(kk-2)+1:Nvar+3*(kk-2)+3) = R*dt^2*(nn-kk+1);
        end
        A(ii+3:ii+5,:) = -A(ii:ii+2,:);
        jj = jj+6;
    end
    
    % Binary variables
    jj = Nvar+Neom+Nslack+Nvar*(N-1)+6*(nn-1)+1;
    A(ii:ii+5,jj:jj+5) = -M*eye(6);
    A(ii+6,jj:jj+5) = ones(1,6);
    
    b(ii:ii+2) = (lbnd-sat.p)-nn*dt*sat.v;
    b(ii+3:ii+5) = -((ubnd-sat.p)-nn*dt*sat.v);
    b(ii+6) = 5;

end

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end