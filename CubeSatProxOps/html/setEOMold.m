function [Aeqnew,beqnew] = setEOM(Aeqold,beqold,sat,scenario)
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
t = scenario.t;

% Add inequality contraints
ii = 1;               %counting no. simulations
Aeq = zeros(3*Nsim,Ntotal);
beq = zeros(3*Nsim,1);
for kk = 1:3:(3*Nsim) %counting position of rows (x,y,z)
    jj = 1;           %counting position of columns (in u())
    
    switch sat.EOM
        case 'HCW'
            A = HCW(scenario);
        case 'LERM'
            A = LERM(scenario,t);
    end
    C = A(4:6,:);

    for nn = 1:ii     %counting no. iterations
        % xdd
        Aeq(kk,jj) = C(1,1)*beta*(ii-nn)+C(1,4)*alpha;      %+x
        Aeq(kk,jj+1) = -Aeq(kk,jj);                         %-x
        Aeq(kk,jj+2) = C(1,2)*beta*(ii-nn)+C(1,5)*alpha;    %+y
        Aeq(kk,jj+3) = -Aeq(kk,jj+2);                       %-y
        Aeq(kk,jj+4) = C(1,3)*beta*(ii-nn)+C(1,6)*alpha;    %+z
        Aeq(kk,jj+5) = -Aeq(kk,jj+4);                       %-z
        Aeq(kk,Nvar+3*(ii-1)+1) = -1;                       %xhcw      
        
        % ydd
        Aeq(kk+1,jj) = C(2,1)*beta*(ii-nn)+C(2,4)*alpha;    %+x
        Aeq(kk+1,jj+1) = -Aeq(kk+1,jj);                     %-x
        Aeq(kk+1,jj+2) = C(2,2)*beta*(ii-nn)+C(2,5)*alpha;  %+y
        Aeq(kk+1,jj+3) = -Aeq(kk+1,jj+2);                   %-y
        Aeq(kk+1,jj+4) = C(2,3)*beta*(ii-nn)+C(2,6)*alpha;  %+z
        Aeq(kk+1,jj+5) = -Aeq(kk+1,jj+4);                   %-z
        Aeq(kk+1,Nvar+3*(ii-1)+2) = -1;                     %yhcw    
        
        % zdd
        Aeq(kk+2,jj) = C(3,1)*beta*(ii-nn)+C(3,4)*alpha;    %+x
        Aeq(kk+2,jj+1) = -Aeq(kk+2,jj);                     %-x
        Aeq(kk+2,jj+2) = C(3,2)*beta*(ii-nn)+C(3,5)*alpha;  %+y
        Aeq(kk+2,jj+3) = -Aeq(kk+2,jj+2);                   %-y
        Aeq(kk+2,jj+4) = C(3,3)*beta*(ii-nn)+C(3,6)*alpha;  %+z
        Aeq(kk+2,jj+5) = -Aeq(kk+2,jj+4);                   %-z
        Aeq(kk+2,Nvar+3*(ii-1)+3) = -1;                     %zhcw
        
        if nn > 1
            % xhcw
            Aeq(kk,Nvar+3*(nn-2)+1) = C(1,1)*(ii-nn+1)*dt^2+C(1,4)*dt;  %x
            Aeq(kk,Nvar+3*(nn-2)+2) = C(1,2)*(ii-nn+1)*dt^2+C(1,5)*dt;  %y
            Aeq(kk,Nvar+3*(nn-2)+3) = C(1,3)*(ii-nn+1)*dt^2+C(1,6)*dt;  %z
            % yhcw
            Aeq(kk+1,Nvar+3*(nn-2)+1) = C(2,1)*(ii-nn+1)*dt^2+C(2,4)*dt;  %x
            Aeq(kk+1,Nvar+3*(nn-2)+2) = C(2,2)*(ii-nn+1)*dt^2+C(2,5)*dt;  %y
            Aeq(kk+1,Nvar+3*(nn-2)+3) = C(2,3)*(ii-nn+1)*dt^2+C(2,6)*dt;  %z
            % zhcw
            Aeq(kk+2,Nvar+3*(nn-2)+1) = C(3,1)*(ii-nn+1)*dt^2+C(3,4)*dt;  %x
            Aeq(kk+2,Nvar+3*(nn-2)+2) = C(3,2)*(ii-nn+1)*dt^2+C(3,5)*dt;  %y
            Aeq(kk+2,Nvar+3*(nn-2)+3) = C(3,3)*(ii-nn+1)*dt^2+C(3,6)*dt;  %z
        end  
        % Update column counter
        jj = jj+6;
    end 
    %Update equality matrix in x,y,z (counters needed for position)
    beq(kk) = -C(1,1)*(x0+ii*dt*vx0)-C(1,2)*(y0+ii*dt*vy0)-C(1,3)*(x0+ii*dt*vz0)...
        -C(1,4)*vx0-C(1,5)*vy0-C(1,6)*vz0;
    beq(kk+1) = -C(2,1)*(x0+ii*dt*vx0)-C(2,2)*(y0+ii*dt*vy0)-C(2,3)*(z0+ii*dt*vz0)...
        -C(2,4)*vx0-C(2,5)*vy0-C(2,6)*vz0;
    beq(kk+2) = -C(3,1)*(x0+ii*dt*vx0)-C(3,2)*(y0+ii*dt*vy0)-C(3,3)*(z0+ii*dt*vz0)...
        -C(3,4)*vx0-C(3,5)*vy0-C(3,6)*vz0;
    
    % Update iteration counter
    ii = ii+1;
    t = t+dt;
end
% Update matrices
Aeqnew = [Aeqold;Aeq];
beqnew = [beqold;beq];
end