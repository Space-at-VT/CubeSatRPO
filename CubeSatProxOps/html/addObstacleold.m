function [Anew,bnew] = addObstacle(Aold,bold,sat,scenario,lbnd,ubnd,N)
x0 = sat.x(end);   y0 = sat.y(end);   z0 = sat.z(end);   %assign positions
vx0 = sat.vx(end); vy0 = sat.vy(end); vz0 = sat.vz(end); %assign velocities
dt = scenario.dt;
Nsim = scenario.Nsim;
Nvar = scenario.Nvar;
Neom = scenario.Neom;
Ntotal = scenario.Ntotal;
NU = 3;
beta = sat.umax*dt^2/sat.m; %velocity multiplier
M = 1e6;

% Safety buffer
d = 0.5;
lbnd = lbnd-d;
ubnd = ubnd+d;

% Add inequality contraints
ii = 1;               %counting no. simulations
A = zeros(Nsim*7,Ntotal);
b = zeros(Nsim*7,1);
for kk = 1:7:(7*Nsim) %counting position of rows (+/-x,y,z...binary)
    jj = 1;           %counting position of columns (in u())
    for nn = 1:ii      %counting no. iterations
        % Postive bounds
        A(kk,jj) = beta*(ii-nn);      %(+x, +x)
        A(kk,jj+1) = -beta*(ii-nn);   %(+x, -x)
        A(kk+1,jj+2) = beta*(ii-nn);  %(+y, +y)
        A(kk+1,jj+3) = -beta*(ii-nn); %(+y, -y)     
        A(kk+2,jj+4) = beta*(ii-nn);  %(+z, +z)
        A(kk+2,jj+5) = -beta*(ii-nn); %(+z, -z)
        
        % Negative bounds
        A(kk+3,jj) = -beta*(ii-nn);   %(-x, +x)
        A(kk+3,jj+1) = beta*(ii-nn);  %(-x, -x)
        A(kk+4,jj+2) = -beta*(ii-nn); %(-y, +y)
        A(kk+4,jj+3) = beta*(ii-nn);  %(-y, -y)
        A(kk+5,jj+4) = -beta*(ii-nn); %(-z, +z)
        A(kk+5,jj+5) = beta*(ii-nn);  %(-z, -z)
        
        if nn > 1
            A(kk,Nvar+3*(nn-2)+1) = dt^2*(ii-nn+1);    %(+x, xhcw)
            A(kk+1,Nvar+3*(nn-2)+2) = dt^2*(ii-nn+1);  %(+y, yhcw)
            A(kk+2,Nvar+3*(nn-2)+3) = dt^2*(ii-nn+1);  %(+z, zhcw)
            A(kk+3,Nvar+3*(nn-2)+1) = -dt^2*(ii-nn+1); %(-x, xhcw)
            A(kk+4,Nvar+3*(nn-2)+2) = -dt^2*(ii-nn+1); %(-y, yhcw)
            A(kk+5,Nvar+3*(nn-2)+3) = -dt^2*(ii-nn+1); %(-z, zhcw)
        end
        jj = jj+6;
    end
    
    % Binary variables
    jj = Nvar+Neom+NU+Nvar*(N-1)+6*(ii-1)+1;
    A(kk,jj) = -M;     %(+x)
    A(kk+1,jj+1) = -M; %(+y)
    A(kk+2,jj+2) = -M; %(+z)
    A(kk+3,jj+3) = -M; %(-x)
    A(kk+4,jj+4) = -M; %(-y)
    A(kk+5,jj+5) = -M; %(-z)
    A(kk+6,jj:jj+5) = ones(1,6);%(+x)
    
    b(kk) = (lbnd(1)-x0)-ii*dt*vx0;      %(+x)
    b(kk+1) = (lbnd(2)-y0)-ii*dt*vy0;    %(+y)
    b(kk+2) = (lbnd(3)-z0)-ii*dt*vz0;    %(+z)
    b(kk+3) = -((ubnd(1)-x0)-ii*dt*vx0); %(-x)
    b(kk+4) = -((ubnd(2)-y0)-ii*dt*vy0); %(-y)
    b(kk+5) = -((ubnd(3)-z0)-ii*dt*vz0); %(-z)
    b(kk+6) = 5;
    
    ii = ii+1;
end

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end