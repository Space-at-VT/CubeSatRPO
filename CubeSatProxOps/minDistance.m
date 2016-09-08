function [Anew,bnew] = minDistance(Aold,bold,sat,scenario,pf)
%% Approach target cost function contraints
x0 = sat.x(end);   y0 = sat.y(end);   z0 = sat.z(end);   %assign positions
vx0 = sat.vx(end); vy0 = sat.vy(end); vz0 = sat.vz(end); %assign velocities
xf = pf(1);  yf = pf(2);  zf = pf(3);
dt = scenario.dt;
Nsim = scenario.Nsim;
Nvar = scenario.Nvar;
Nhcw = scenario.Nhcw;
Ntotal = scenario.Ntotal;
beta = sat.umax*dt^2/sat.m; %velocity multiplier

% Target objective absolute value
A = zeros(6,Ntotal);
b = zeros(6,1);
jj = 1;         %counting position of columns (in u())
for nn = 1:Nsim %counting no. iterations
    A(1,jj) = beta*(Nsim-nn);    %(+x, +x)
    A(1,jj+1) = -beta*(Nsim-nn); %(+x, -x)
    
    A(2,jj+2) = beta*(Nsim-nn);  %(+y, +y)
    A(2,jj+3) = -beta*(Nsim-nn); %(+y, -y)
    
    A(3,jj+4) = beta*(Nsim-nn);  %(+z, +z)
    A(3,jj+5) = -beta*(Nsim-nn); %(+z, -z)
    
    A(4,jj) = -beta*(Nsim-nn);   %(-x, +x)
    A(4,jj+1) = beta*(Nsim-nn);  %(-x, -x)
    
    A(5,jj+2) = -beta*(Nsim-nn); %(-y, +y)
    A(5,jj+3) = beta*(Nsim-nn);  %(-y, -y)
    
    A(6,jj+4) = -beta*(Nsim-nn); %(-z, +z)
    A(6,jj+5) = beta*(Nsim-nn);  %(-z, -z)
    
    if nn > 1
        A(1,Nvar+3*(nn-2)+1) = dt^2*(Nsim-nn+1);  %(+x, xhcw)
        A(2,Nvar+3*(nn-2)+2) = dt^2*(Nsim-nn+1);  %(+y, yhcw)
        A(3,Nvar+3*(nn-2)+3) = dt^2*(Nsim-nn+1);  %(+z, zhcw)
        A(4,Nvar+3*(nn-2)+1) = -dt^2*(Nsim-nn+1); %(-x, xhcw)
        A(5,Nvar+3*(nn-2)+2) = -dt^2*(Nsim-nn+1); %(-y, yhcw)
        A(6,Nvar+3*(nn-2)+3) = -dt^2*(Nsim-nn+1); %(-z, zhcw)
    end       
    jj = jj+6;
end

A(1,Nvar+Nhcw+1) = -1; %(+x, xtarget)
A(2,Nvar+Nhcw+2) = -1; %(+y, ytarget)
A(3,Nvar+Nhcw+3) = -1; %(+z, ztarget)
A(4,Nvar+Nhcw+1) = -1; %(-x, xtarget)
A(5,Nvar+Nhcw+2) = -1; %(-y, ytarget)
A(6,Nvar+Nhcw+3) = -1; %(-z, ztarget)

b(1) = -x0+xf-Nsim*dt*vx0;    %(+x)
b(2) = -y0+yf-Nsim*dt*vy0;    %(+y)
b(3) = -z0+zf-Nsim*dt*vz0;    %(+z)
b(4) = -(-x0+xf-Nsim*dt*vx0); %(-x)
b(5) = -(-y0+yf-Nsim*dt*vy0); %(-y)
b(6) = -(-z0+zf-Nsim*dt*vz0); %(-z)

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end