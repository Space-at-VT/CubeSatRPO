function [Anew,bnew] = holdProximity(Aold,bold,sat,scenario,lbnd,ubnd)
x0 = sat.x(end);   y0 = sat.y(end);   z0 = sat.z(end);   %assign positions
vx0 = sat.vx(end); vy0 = sat.vy(end); vz0 = sat.vz(end); %assign velocities
dt = scenario.dt;
Nsim = scenario.Nsim;
Nvar = scenario.Nvar;
Ntotal = scenario.Ntotal;
beta = sat.umax*dt^2/sat.m; %velocity multiplier

% Add inequality contraints
ii = 1;                %counting no. simulations
A = zeros(6*Nsim,Ntotal);
b = zeros(6*Nsim,1);
for kk = 1:6:(6*Nsim); %counting position of rows (+/-x,y,z)
    jj = 1;            %counting position of columns (in u())
    for nn = 1:ii       %counting no. iterations
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
        A(kk+4,jj+2) = -beta*(ii-nn); %(-x, +y)
        A(kk+4,jj+3) = beta*(ii-nn);  %(-x, -y)
        A(kk+5,jj+4) = -beta*(ii-nn); %(-x, +z)
        A(kk+5,jj+5) = beta*(ii-nn);  %(-x, -z)
        
        %HCW
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
    
    b(kk) = (ubnd(1)-x0)-ii*dt*vx0;   %(+x)
    b(kk+1) = (ubnd(2)-y0)-ii*dt*vy0; %(+y)
    b(kk+2) = (ubnd(3)-z0)-ii*dt*vz0; %(+z)
    b(kk+3) = -((lbnd(1)-x0)-ii*dt*vx0); %(-x)
    b(kk+4) = -((lbnd(2)-y0)-ii*dt*vy0); %(-y)
    b(kk+5) = -((lbnd(3)-z0)-ii*dt*vz0); %(-z)
    
    ii = ii+1;
end

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end