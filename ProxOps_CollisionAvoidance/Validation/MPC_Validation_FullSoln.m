%% RPO_MILP_MPC(p0,v0,m) is a simulation of a 6U cubesat in proximity operation
% around a larger chief satellite. Model predictive control is used in
% conjunction with mixed integer linear programming to generate the cubesat
% trajectory optimizing primarily for fuel once the desired target position 
% is achieved. The controller cycles through different phases to maintain 
% proximity to the chief while avoiding collision and minimizing fuel usage. 
% Uses HCW equations of relative motion in simulation. Thruster constraint to 
% toggle on/off.
%
% [x,v,u,t] = RPO_MILP_MPC(p0,v0,m) starts at initial state x0 and 
% v0 and finds a minimum fuel trajectory. The position x, velocity v, and 
% control thrusts u are output along with each time step t. 
%
% Parameters
% T:           Horizon time (s), maximum time for each MPC iteration.
% dt:          Time step (s).
% a:           Semi-major axis (m).
% x0,y0,z0:    Initial position (m).
% vx0,vy0,vz0: Initial velocity (m/s).
% vmax:        Maximum velocity in each axis (m/s).
% umax:        Maximum thrust (N).
% xbmin,xbmax: Lower and upper bounds for obstacles (m).
% d:           Safety buffer distance from obstacles to prevent clipping (m).
% dispIter:    Toggle iteration data display.
%
% Outputs
% Plot of trajectory and obstacles in 3D space.
% Plots of control signals, velocity, and position vs time.

function MPC_Validation_FullSoln
%% Inputs
clc, close all, clear all

% Orbital State
a = 10e6; %m

% Relative Initial state
x0 = 0; %m
y0 = 7;
z0 = 0;
vx0 = 0; %m/s
vy0 = 0;
vz0 = 0;

% Vehicle parameters
vmax = 0.5;  %m/s
umax = 0.26;  %N
mDry = 13;   %kg
mFuel = 0.5; %kg
I = 800;   %s

% Obstacles bounds [m]
xbmin = -1;
ybmin = -1;
zbmin = -1;
xbmax = 1;
ybmax = 1;
zbmax = 1;

% Safety buffer [m]
d = 1;

% Optmization weights
w1 = 1e-1; %Thrust
w2 = 1;    %Targeting
w3 = 1e3;  %Min time

% Options
dispIter = 1;

%% Pre-MPC setup
% Proximity box [m]
xub = 10;
xlb = xbmax+2*d;
yub = ybmax;
ylb = ybmin;
zub = zbmax;
zlb = zbmin;

% Approach Target, center of proximity box
xf = 0;
yf = -30;
zf = 0;
% xf = (xub+xlb)/2;
% yf = (yub+ylb)/2;
% zf = (zub+zlb)/2;

% Add safety buffer, surrounding obstacle [m]
xbmind = xbmin-d;
ybmind = ybmin-d;
zbmind = zbmin-d;
xbmaxd = xbmax+d;
ybmaxd = ybmax+d;
zbmaxd = zbmax+d;

% Simulation time [s]
tmax = 50;

% Mean motion and period
mu = 3.986004418e14; %m^3s^2
n = sqrt(mu/a^3);    %mean motion [s^-1]
TP = 2*pi/n;         %period

% Mass flow rate [kg/s]
g0 = 9.80665;
ISP = I/(0.5*g0); %Full tank mass ~0.5
mdot = umax/ISP/g0;

% Initial state
x = x0;     y = y0;     z = z0;     %positions
vx = vx0;   vy = vy0;   vz = vz0;   %velocities
t0 = 0;

% Horizon setup
tol = 1;              %tolerance for Approach target [m]
m = mDry+mFuel;       %total mass
phase = 'Approach';   %setting Approach sequence
iter = 1;             %number of simulations
tic                   %start timer

% Counter variables
out = 0;
timeout = 0;
%while  iter == 1; %mFuel > 0  %computation will end when fuel mass is 0
    %% Optimization setup    
    % Introduce matrices/vectors
    Aeq = [];beq = []; %equality constraints
    A = [];b = [];     %inequality constraints
    
    % Target check
    % If the deputy is within the tolerant distance of the target
    % position, the phase will move to the Hold sequence
    if strcmp(phase,'Approach')
        if abs(x(iter)-xf) <= tol && abs(y(iter)-yf) <= tol && abs(z(iter)-zf) <= tol
            phase = 'Hold';
        end
    end
    
    % End of life maneuver to avoid collision with chief
    if mFuel < 0.001
        phase = 'Approach';
        xf = 20;
    end
    
    % Objective behavior
    switch phase
        % Approach - deputy minimizes distance between itself and the center
        % of the proxmity area while avoiding collision. Used typically at
        % the start of the simulation to approach target and when the target
        % temporarily leaves the proximity area and needs to return.
        case 'Approach' 
            T = tmax; %horizon time [s]
            dt = 1; %time step [s]
            t = t0:dt:t0+T; %total time trajectory is generated for 

            % Number of variables
            NObj = 1;           %objects to avoid
            Nsim = length(t)-1; %simulations
            Nvar = 6*Nsim;      %positions & velocities
            Nhcw = 3*Nsim;      %HCW accelerations
            NU = 3+3*Nsim;      %targeting (min distance)
            Nbi = NObj*Nvar;    %binary (min time)
            Ntotal = Nvar+Nhcw+NU+Nbi;

            % Parameter bounds, lower & upper
            lb = [zeros(Nvar,1);   %Control thrusts
                -inf*ones(Nhcw,1); %HCW accelerations
                zeros(3,1);        %Target distance
                zeros(3*Nsim,1);   %Min time for last iteration
                zeros(Nbi,1)];     %Collision avoidance
            
            ub = [ones(Nvar,1);   %Control thrusts
                inf*ones(Nhcw,1); %HCW accelerations
                inf*ones(3,1);    %Target distance
                ones(3*Nsim,1);   %Min time for last iteration
                ones(Nbi,1)];     %Collision avoidance
            
            % Function coefficients
            f = [w1*dt*ones(Nvar,1); %Control thrusts
                zeros(Nhcw,1);       %HCW accelerations
                w2*ones(3,1);        %Target distance
                w3*ones(3*Nsim,1);   %Min time for last iteration
                zeros(Nbi,1)];       %Collision avoidance
            
            % Integer constraints
            intcon = [1:Nvar,Nvar+Nhcw+3+1:Ntotal];
            
            % Min distance constraint
            [A,b] = Target(A,b,umax,[x(end),y(end),z(end)],[vx(end),vy(end),vz(end)],...
                [xf,yf,zf],dt,m,Nsim,Ntotal);
            
            % Min time constraint
%            [A,b] = MinTime(A,b,umax,[x(end),y(end),z(end)],[vx(end),vy(end),vz(end)],...
%                [xf,yf,zf],dt,m,Nsim,Ntotal);
            
            % Obstacle contraints
            for N = 1:NObj
                [A,b] = NewObstacle(N,A,b,umax,[x(end),y(end),z(end)],[vx(end),vy(end),vz(end)],...
                    dt,m,Nsim,Nbi,xbmind(N),xbmaxd(N),ybmind(N),ybmaxd(N),zbmind(N),zbmaxd(N));
            end
             
        % Hold - cubesat is contrained to remain in the proximity area and
        % has sole objective to minimize fuel.
%         case 'Hold'
%             T = 40;
%             dt = 1;
%             t = t0:dt:t0+T;
% 
%             % Number of variables
%             Nsim = length(t)-1; %simulations
%             Nvar = 6*Nsim;      %positions & velocities
%             Nhcw = 3*Nsim;      %HCW accelerations
%             NU = 0;             %targeting- no target in Hold sequence
%             Nbi = 0;            %timing- no min time in Hold sequence
%             Ntotal = Nvar+Nhcw+NU+Nbi;     
%             
%             % Parameter bounds, lower & upper
%             lb = [zeros(Nvar,1);    %Control thrusts
%                 -inf*ones(Nhcw,1)]; %HCW accelerations
%             
%             ub = [ones(Nvar,1);    %Control thrusts
%                 inf*ones(Nhcw,1)]; %HCW accelerations
%             
%             % Function coefficients
%             f = [dt*ones(Nvar,1); %Control thrusts
%                 zeros(Nhcw,1)];   %HCW accelerations
%             
%             % Integer constraints
%             intcon = 1:Nvar;
%             
%             % Proximity constraint (to stay within bounds)
%             [A,b] = Proximity(A,b,umax,[x(end),y(end),z(end)],[vx(end),vy(end),vz(end)],...
%                 dt,m,Nsim,Ntotal,xlb,xub,ylb,yub,zlb,zub);

     end

    % HCW accelerations
    [Aeq,beq] = HCW(Aeq,beq,umax,[x(end),y(end),z(end)],[vx(end),vy(end),vz(end)],...
        dt,m,n,Nsim,Ntotal);
     
    % Max velocity constraint
    [A,b] = MaxVelocity(A,b,umax,[vx(end),vy(end),vz(end)],dt,m,Nsim,Ntotal,vmax);
    
       
    %% MILP Optimization
    % Options set to hide default solver display and limit the maximum time
    % spent optimizing solution to maintain realtime capability
    options = optimoptions(@intlinprog,'Display','iter','MaxTime',60); 
    [u,fval,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub,options);
    fval
    
    %% Post process
    % During Hold sequence, if deputy leaves proximity bounds, phase will
    % return to Approach to re-enter bounds after which it will revert back
    % to Hold sequence
    if isempty(fval) %Deputy leaves bounds
        phase = 'Approach';
        out = out+1;
    else    
        % Solver timeout
        if exitflag == 2;
            timeout = timeout+1;
        end
        
        % Thrust controls - pos/neg values of control thrust in each axis
        uxT = umax*(u(1:6:Nvar)-u(2:6:Nvar));
        uyT = umax*(u(3:6:Nvar)-u(4:6:Nvar));
        uzT = umax*(u(5:6:Nvar)-u(6:6:Nvar));
        ux(iter) = uxT(1);
        uy(iter) = uyT(1);
        uz(iter) = uzT(1);
        
        % HCW - assigning hcw accelerations
        uhxT = u(Nvar+1:3:Nvar+Nhcw);
        uhyT = u(Nvar+2:3:Nvar+Nhcw);
        uhzT = u(Nvar+3:3:Nvar+Nhcw);
        uhx(iter) = uhxT(1);
        uhy(iter) = uhyT(1);
        uhz(iter) = uhzT(1);
        
        for qq = 1:length(uxT);
            vx(qq+1) = vx(qq)+(uxT(qq)/m+uhxT(qq))*dt;
            x(qq+1) = x(qq)+vx(qq)*dt;
            vy(qq+1) = vy(qq)+(uyT(qq)/m+uhyT(qq))*dt;
            y(qq+1) = y(qq)+vy(qq)*dt;
            vz(qq+1) = vz(qq)+(uzT(qq)/m+uhzT(qq))*dt;
            z(qq+1) = z(qq)+vz(qq)*dt;
        end
        
        % Update position & velocity    
%         vx(iter+1) = vx(iter)+(ux(iter)/m+uhx(iter))*dt;
%         x(iter+1) = x(iter)+vx(iter)*dt;
%         vy(iter+1) = vy(iter)+(uy(iter)/m+uhy(iter))*dt;
%         y(iter+1) = y(iter)+vy(iter)*dt;
%         vz(iter+1) = vz(iter)+(uz(iter)/m+uhz(iter))*dt;
%         z(iter+1) = z(iter)+vz(iter)*dt;
%         
        % Update fuel mass
        mFuel = mFuel-mdot*dt*sum(abs([uxT(1),uyT(1),uzT(1)])); %calculate fuel loss based on number of times controls were turned on (mult. by dt=1s)
        m = mDry+mFuel;        
                
        % Calculate cpu time
        cpuT(iter) = toc;
        
        % Data display
        cpustr = datestr(cpuT(iter)/3600/24, 'DD HH:MM:SS');
        simstr = datestr(t0/3600/24, 'DD HH:MM:SS');
        if dispIter
            fprintf('Iteration %d\n',iter)  
            fprintf('--------------------------------\n')
            fprintf('Simulation time: %s\n',simstr) 
            fprintf('CPU time: %s\n',cpustr)
            fprintf('Fuel remaining: %7.4f kg\n',mFuel)
            fprintf('x: %6.3f m  vx: %6.3f m/s\n',x(end),vx(end))
            fprintf('y: %6.3f m  vy: %6.3f m/s\n',y(end),vy(end))
            fprintf('z: %6.3f m  vz: %6.3f m/s\n\n',z(end),vz(end))
            fprintf('Proximity bound exits: %d\n',out)
            fprintf('Solver timeouts: %d\n',timeout)
        end

        % Update time and iteration
        t0 = t0+T;
        iter = iter+1;
    end
    
    % Catch max time
%    if t0 > tmax,break,end
%end

% Post MPC processing - fill last control
uxT = [uxT;0];
uyT = [uyT;0];
uzT = [uzT;0];

% Final time
t = 0:dt:t0;

save('FullSoln_Approach.mat','t','x','y','z','uxT','uyT','uzT','fval','cpuT')


%% Plots
% Trajectory
figure(1)
hold on
p1 = plot3(x,y,z,'b','linewidth',2);
p2 = plot3(x0,y0,z0,'k^','linewidth',2,'markersize',10);
p4 = quiver3(x,y,z,-uxT',-uyT',-uzT',1,'r','linewidth',2);
p5 = PlotObstacle(xbmin,xbmax,ybmin,ybmax,zbmin,zbmax,'k');
p6 = PlotObstacle(xlb,xub,ylb,yub,zlb,zub,'--k');
hold off
grid on
xlabel('Radial, x [m]')
ylabel('In-track, y [m]')
zlabel('Cross-track, z [m]')
title('Deputy Trajectory in Space')
legend([p5,p6,p1,p2,p4],{'Chief','Proximity Bounds','Deputy Trajectory','x0','Thrust'},'location','BestOutside')
axis('equal')
camva(8)
view(-45,15)

% Controls
figure(2)
subplot(3,3,1)
hold on
stairs(t,uxT/umax,'-b','linewidth',2)
plot([0 t0],[0 0],'--k','linewidth',2)
axis([0 t0 -1.5 1.5])
grid on
title('Control Signals vs Time')
ylabel('ux [N]')

subplot(3,3,4)
hold on
stairs(t,uyT/umax,'-r','linewidth',2)
plot([0 t0],[0 0],'--k','linewidth',2)
axis([0 t0 -1.5 1.5])
grid on
ylabel('uy [N]')

subplot(3,3,7)
hold on
stairs(t,uzT/umax,'-g','linewidth',2)
plot([0 t0],[0 0],'--k','linewidth',2)
axis([0 t0 -1.5 1.5])
grid on
xlabel('Time [s]')
ylabel('uz [N]')

% Velocity
subplot(3,3,2)
plot(t,vx,'-b','linewidth',2)
axis([0 t0 0 1],'auto y')
grid on
ylabel('Vx [m/s]')
title('Velocity vs Time')

subplot(3,3,5)
plot(t,vy,'-r','linewidth',2)
axis([0 t0 0 1],'auto y')
grid on
ylabel('Vy [m/s]')

subplot(3,3,8)
plot(t,vz,'-g','linewidth',2)
axis([0 t0 0 1],'auto y')
grid on
xlabel('Time [s]')
ylabel('Vz [m/s]')

% Position
subplot(3,3,3)
plot(t,x,'-b','linewidth',2)
axis([0 t0 0 1],'auto y')
grid on
ylabel('x [m]')
title('Position vs Time')

subplot(3,3,6)
plot(t,y,'-r','linewidth',2)
axis([0 t0 0 1],'auto y')
grid on
ylabel('y [m]')

subplot(3,3,9)
plot(t,z,'-g','linewidth',2)
axis([0 t0 0 1],'auto y')
grid on
xlabel('Time [s]')
ylabel('z [m]')
end

function [Aeqnew,beqnew] = HCW(Aeqold,beqold,umax,p0,v0,dt,m,n,Nsim,Ntotal)
%% HCW equations
x0 = p0(1);  y0 = p0(2);  z0 = p0(3);  %assign positions
vx0 = v0(1); vy0 = v0(2); vz0 = v0(3); %assign velocities
Nvar = 6*Nsim;
alpha = umax*dt/m;  %position multiplier
beta = umax*dt^2/m; %velocity multiplier

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

function [Anew,bnew] = MaxVelocity(Aold,bold,umax,v0,dt,m,Nsim,Ntotal,vmax)
%% Obstacles inequalities
vx0 = v0(1); vy0 = v0(2); vz0 = v0(3);
alpha = umax*dt/m;
Nvar = 6*Nsim;

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

function [Anew,bnew] = Target(Aold,bold,umax,p0,v0,pf,dt,m,Nsim,Ntotal)
%% Approach target cost function contraints
x0 = p0(1);  y0 = p0(2);  z0 = p0(3);
vx0 = v0(1); vy0 = v0(2); vz0 = v0(3);
xf = pf(1);  yf = pf(2);  zf = pf(3);
Nvar = 6*Nsim;
Nhcw = 3*Nsim;
beta = umax*dt^2/m;

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

function [Anew,bnew] = MinTime(Aold,bold,umax,p0,v0,pf,dt,m,Nsim,Ntotal)
%% Minimum time cost function constraints
x0 = p0(1);  y0 = p0(2);  z0 = p0(3);
vx0 = v0(1); vy0 = v0(2); vz0 = v0(3);
xf = pf(1);  yf = pf(2);  zf = pf(3);
Nvar = 6*Nsim;
Nhcw = 3*Nsim;
beta = umax*dt^2/m;
M = 1e6; %binary multiplier

% Min time objective absolute value
A = zeros(6*Nsim,Ntotal);
b = zeros(6*Nsim,1);
ii = 1;               %counting no. simulations
for kk = 1:6:(6*Nsim) %counting position of rows (+/-x,y,z)
    jj = 1;           %counting position of columns (in u())
    for nn = 1:ii      %counting no. iterations
        A(kk,jj) = beta*(ii-nn);      %(+x, +x)
        A(kk,jj+1) = -beta*(ii-nn);   %(+x, -x)        
        A(kk+1,jj+2) = beta*(ii-nn);  %(+y, +y)
        A(kk+1,jj+3) = -beta*(ii-nn); %(+y, -y)
        A(kk+2,jj+4) = beta*(ii-nn);  %(+z, +z)
        A(kk+2,jj+5) = -beta*(ii-nn); %(+z, -z)
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
    A(kk,Nvar+Nhcw+3*Nsim+1) = -M;   %(+x)
    A(kk+1,Nvar+Nhcw+3*Nsim+2) = -M; %(+y)
    A(kk+2,Nvar+Nhcw+3*Nsim+3) = -M; %(+z)
    A(kk+3,Nvar+Nhcw+3*Nsim+1) = -M; %(-x)
    A(kk+4,Nvar+Nhcw+3*Nsim+2) = -M; %(-y)
    A(kk+5,Nvar+Nhcw+3*Nsim+3) = -M; %(-z)
    
    b(kk) = -x0+xf-ii*dt*vx0;      %(+x)
    b(kk+1) = -y0+yf-ii*dt*vy0;    %(+y)
    b(kk+2) = -z0+zf-ii*dt*vz0;    %(+z)
    b(kk+3) = -(-x0+xf-ii*dt*vx0); %(-x)
    b(kk+4) = -(-y0+yf-ii*dt*vy0); %(-y)
    b(kk+5) = -(-z0+zf-ii*dt*vz0); %(-z)
    
    ii = ii+1;
end

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end

function [Anew,bnew] = NewObstacle(N,Aold,bold,umax,p0,v0,dt,m,Nsim,Ntotal,xbmind,xbmaxd,ybmind,ybmaxd,zbmind,zbmaxd)
%% Obstacles inequalities
x0 = p0(1);  y0 = p0(2);  z0 = p0(3);
vx0 = v0(1); vy0 = v0(2); vz0 = v0(3);
Nvar = 6*Nsim;
Nhcw = 3*Nsim;
NU = 3+3*Nsim;
beta = umax*dt^2/m;
M = 1e6;

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
    jj = Nvar+Nhcw+NU+Nvar*(N-1)+6*(ii-1)+1;
    A(kk,jj) = -M;     %(+x)
    A(kk+1,jj+1) = -M; %(+y)
    A(kk+2,jj+2) = -M; %(+z)
    A(kk+3,jj+3) = -M; %(-x)
    A(kk+4,jj+4) = -M; %(-y)
    A(kk+5,jj+5) = -M; %(-z)
    A(kk+6,jj:jj+5) = ones(1,6);%(+x)
    
    b(kk) = (xbmind-x0)-ii*dt*vx0;      %(+x)
    b(kk+1) = (ybmind-y0)-ii*dt*vy0;    %(+y)
    b(kk+2) = (zbmind-z0)-ii*dt*vz0;    %(+z)
    b(kk+3) = -((xbmaxd-x0)-ii*dt*vx0); %(-x)
    b(kk+4) = -((ybmaxd-y0)-ii*dt*vy0); %(-y)
    b(kk+5) = -((zbmaxd-z0)-ii*dt*vz0); %(-z)
    b(kk+6) = 5;
    
    ii = ii+1;
end

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end

function [Anew,bnew] = Proximity(Aold,bold,umax,p0,v0,dt,m,Nsim,Ntotal,xlb,xub,ylb,yub,zlb,zub)
%% Obstacles inequalities
x0 = p0(1);  y0 = p0(2);  z0 = p0(3);
vx0 = v0(1); vy0 = v0(2); vz0 = v0(3);
Nvar = 6*Nsim;
beta = umax*dt^2/m;

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
    
    b(kk) = (xub-x0)-ii*dt*vx0;   %(+x)
    b(kk+1) = (yub-y0)-ii*dt*vy0; %(+y)
    b(kk+2) = (zub-z0)-ii*dt*vz0; %(+z)
    b(kk+3) = -((xlb-x0)-ii*dt*vx0); %(-x)
    b(kk+4) = -((ylb-y0)-ii*dt*vy0); %(-y)
    b(kk+5) = -((zlb-z0)-ii*dt*vz0); %(-z)
    
    ii = ii+1;
end

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end

function p = PlotObstacle(xbmin,xbmax,ybmin,ybmax,zbmin,zbmax,style)
%% Plot obstacle
xb = [xbmin xbmax xbmax xbmin xbmin];
yb = [ybmin ybmin ybmax ybmax ybmin];
zbl = [zbmin zbmin zbmin zbmin zbmin];
zbu = [zbmax zbmax zbmax zbmax zbmax];

figure(1)
hold on
p = plot3(xb,yb,zbl,style,'linewidth',2);
plot3(xb,yb,zbu,style,'linewidth',2)
for i = 1:4
    plot3([xb(i) xb(i)],[yb(i) yb(i)],[zbmin,zbmax],style,'linewidth',2)
end
hold off
end