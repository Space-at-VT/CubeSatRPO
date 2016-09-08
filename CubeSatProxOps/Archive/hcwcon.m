function [Aeqnew,beqnew] = hcwcon(Aeqold,beqold,sat,scenario)
%% HCW equations
x0 = sat.x(end);   y0 = sat.y(end);   z0 = sat.z(end);   %assign positions
vx0 = sat.vx(end); vy0 = sat.vy(end); vz0 = sat.vz(end); %assign velocities
n = scenario.n;
dt = scenario.dt;
Nsim = scenario.Nsim;
Nvar = scenario.Nvar;
Ntotal = scenario.Ntotal;
alpha = sat.umax*dt/sat.m;  %position multiplier
beta = sat.umax*dt^2/sat.m; %velocity multiplier

% Add inequality contraints
ii = 1;               %counting no. simulations
Aeq = zeros(3*Nsim,Ntotal);
beq = zeros(3*Nsim,1);
for kk = 1:3:(3*Nsim) %counting position of rows (x,y,z)
    jj = 1;           %counting position of columns (in u())    
    %(_,_) denotes which cell of matrix is populated
    for nn = 1:ii     %counting no. iterations
        % xdd = 3n^2x+2nyd
        Aeq(kk,jj) = (3*n^2)*beta*(ii-nn);    %(x, +x)
        Aeq(kk,jj+1) = (-3*n^2)*beta*(ii-nn); %(x, -x)
        Aeq(kk,jj+2) = 2*n*alpha;             %(x, +y)
        Aeq(kk,jj+3) = -2*n*alpha;            %(x, -y)
        Aeq(kk,Nvar+3*(ii-1)+1) = -1;         %(x, xhcw)      
        
        % ydd = -2*n*xd
        Aeq(kk+1,jj) = -2*n*alpha;      %(y, +x)
        Aeq(kk+1,jj+1) = 2*n*alpha;     %(y, -x)
        Aeq(kk+1,Nvar+3*(ii-1)+2) = -1; %(y, yhcw)
        
        % zdd = -n^2z
        Aeq(kk+2,jj+4) = -n^2*beta*(ii-nn); %(z, +z)
        Aeq(kk+2,jj+5) = n^2*beta*(ii-nn);  %(z, -z)
        Aeq(kk+2,Nvar+3*(ii-1)+3) = -1;     %(z, zhcw)
        
        if nn > 1
            Aeq(kk,Nvar+3*(nn-2)+1) = (ii-nn+1)*(3*n^2*dt^2);  %(x,xhcw)
            Aeq(kk,Nvar+3*(nn-2)+2) = 2*n*dt;                  %(x,yhcw)       
            Aeq(kk+1,Nvar+3*(nn-2)+1) = -2*n*dt;               %(y,xhcw)                   
            Aeq(kk+2,Nvar+3*(nn-2)+3) = (ii-nn+1)*(-n^2*dt^2); %(z,zhcw)
        end  
        % Update column counter
        jj = jj+6;
    end 
    %Update equality matrix in x,y,z (counters needed for position)
    beq(kk) = 3*n^2*(-x0-ii*dt*vx0)-2*n*(vy0);
    beq(kk+1) = 2*n*vx0;    
    beq(kk+2) = -n^2*(-z0-ii*dt*vz0);
    
    % Update iteration counter
    ii = ii+1;
end
% Update matrices
Aeqnew = [Aeqold;Aeq];
beqnew = [beqold;beq];
end