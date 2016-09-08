%function [Anew,bnew] = maxVelocity(Aold,bold,umax,v0,dt,m,Nsim,Ntotal,vmax)
function [Anew,bnew] = maxVelocity(Aold,bold,sat,scenario)
%% Obstacles inequalities
vx0 = sat.vx(end); vy0 = sat.vy(end); vz0 = sat.vz(end); %assign velocities
vmax = sat.vmax;
dt = scenario.dt;
Nsim = scenario.Nsim;
Nvar = scenario.Nvar;
Ntotal = scenario.Ntotal;
alpha = sat.umax*dt/sat.m;  %position multiplier

% Add inequality contraints
ii = 1;               %counting no. simulations
A = zeros(6*Nsim,Ntotal);
b = zeros(6*Nsim,1);
for kk = 1:6:(6*Nsim) %counting position of rows (+/-x,y,z)
    jj = 1;           %counting position of columns (in u())
    for nn = 1:ii     %counting no. iterations
        % Postive bounds
        A(kk,jj) = alpha;      %(+x, +x)
        A(kk,jj+1) = -alpha;   %(+x, -x)
        A(kk+1,jj+2) = alpha;  %(+y, +y)
        A(kk+1,jj+3) = -alpha; %(+y, -y)      
        A(kk+2,jj+4) = alpha;  %(+z, +z)
        A(kk+2,jj+5) = -alpha; %(+z, -z)
      
        % Negative bounds
        A(kk+3,jj) = -alpha;   %(-x, +x)
        A(kk+3,jj+1) = alpha;  %(-x, -x)
        A(kk+4,jj+2) = -alpha; %(-y, +y)
        A(kk+4,jj+3) = alpha;  %(-y, -y)
        A(kk+5,jj+4) = -alpha; %(-z, +z)
        A(kk+5,jj+5) = alpha;  %(-z, -z)
        
        % HCW
        if nn > 1
            A(kk,Nvar+3*(nn-2)+1) = dt;    %(+x, xhcw)
            A(kk+1,Nvar+3*(nn-2)+2) = dt;  %(+y, yhcw)
            A(kk+2,Nvar+3*(nn-2)+3) = dt;  %(+z, zhcw)
            A(kk+3,Nvar+3*(nn-2)+1) = -dt; %(-x, xhcw)
            A(kk+4,Nvar+3*(nn-2)+2) = -dt; %(-y, yhcw)
            A(kk+5,Nvar+3*(nn-2)+3) = -dt; %(-z, zhcw)
        end     
        jj = jj+6;
    end
    % Binary variables - minimizing velocity
    b(kk) = (vmax-vx0);   %(+x)
    b(kk+1) = (vmax-vy0); %(+y)
    b(kk+2) = (vmax-vz0); %(+z)
    b(kk+3) = (vmax+vx0); %(-x)
    b(kk+4) = (vmax+vy0); %(-y)
    b(kk+5) = (vmax+vz0); %(-z)
    
    ii = ii+1;
end

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end