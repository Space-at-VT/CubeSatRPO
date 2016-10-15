function phaseManeuver()
clc
close all

scenario = newScenario;
scenario.a = 7000e3;
tf = scenario.TP*3/4;
scenario.tmax = tf;
scenario.T = tf;
scenario.dt = 10;
scenario.ecc = 0.0;

sat = newSatellite;
sat.umax = 0.25;
sat.EOM = 'LERM';
sat.x = 0;
sat.y = -500;
sat.z = -200;
sat.vy = -2*scenario.n*sat.x;

x0 = 100;
Xf = [x0,0,0,0,-2*scenario.n*x0,-0.1];

Nvar = scenario.Nvar;
Neom = scenario.Neom;
dt = scenario.dt;

% Function coefficients
f = [dt*ones(Nvar,1); %Control thrusts
    zeros(Neom,1);
    zeros(3,1)];

% Parameter bounds, lower & upper
lb = [zeros(Nvar,1);   %Control thrusts
    -inf*ones(Neom,1);
     zeros(3,1)];

ub = [ones(Nvar,1);   %Control thrusts
    inf*ones(Neom,1);
    ones(3,1)];

scenario.Ntotal;
Nsim = scenario.Nsim;

% Equality contraints
Aeq = []; beq = [];
[Aeq,beq] = setEOMtest(Aeq,beq,sat,scenario);
[Aeq,beq] = setPhaseState(Aeq,beq,sat,scenario,Xf);

% Inequality contraints
A = [];   b = [];

intcon = [];

% options = optimoptions(@linprog);
[U,fval,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub);

u = zeros(3,Nsim);
a = zeros(3,Nsim);
v = zeros(3,Nsim);
x = zeros(3,Nsim);
v(:,1) = sat.v;
x(:,1) = sat.p;

jj = 1;
kk = 1;
for ii = 1:Nsim
    u(:,ii) = sat.umax*(U(jj:2:jj+5)-U(jj+1:2:jj+5));
    a(:,ii) = U(Nvar+kk:Nvar+kk+2);
    v(:,ii+1) = v(:,ii)+(u(:,ii)/sat.m+a(:,ii))*dt;
    x(:,ii+1) = x(:,ii)+v(:,ii)*dt;
    u(:,ii+1) = zeros(3,1);
    jj = jj+6;
    kk = kk+3;
end

u

% Impulse correction
tt = dt*ones(3,Nsim);
for ii = 1:3
    for jj = 1:Nsim
        if u(ii,jj) ~= 0 && u(ii,jj) < 0.99*sat.umax || u(ii,jj) > 0.99*sat.umax
            tt(ii,jj) = u(ii,jj)*dt/sat.umax;
        end
    end
end

dfuel = sum(sum(abs(u)))*sat.mdot
sat.fuel = sat.fuel-sum(abs(u),2)*sat.mdot;

%figure
hold on
plot3(x(1,:),x(2,:),x(3,:),'-b','linewidth',1.5)
quiver3(x(1,:),x(2,:),x(3,:),-u(1,:),-u(2,:),-u(3,:),'r')
hold off
grid on
axis('equal','vis3d','tight')
xlabel('x')
ylabel('y')
zlabel('z')
view(30,30)

%% ode45
x0 = x(:,end); %m
v0 = v(:,end); %m/s
orbit(x0,v0,scenario)

end

function orbit(x0,v0,scenario)
%% ode45
tf = scenario.TP; %s

% ODE TPsolver
options = odeset('RelTol',1e-5);
[t,X] = ode45(@(t,x)HCW(t,x,scenario.n),[0;tf],[x0;v0],options);

% Post process
x = X(:,1);
y = X(:,2);
z = X(:,3);

hold on
plot3(x,y,z,'-b','linewidth',1.5)
hold off
grid on
axis('equal','vis3d','tight')
xlabel('x')
ylabel('y')
zlabel('z')
view(30,30)

end

function DX = HCW(t,X,n)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% All units are in meters and meters/sec, instead of mu = 3.986e5 km^3/s^3,
% it is 3.986e14 m^3/s^2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code is provided 'as-is' for use with AOE 5234: Orbital Mechanics.
% Numerical accuracy is guaranteed only to within the bounds specified by
% The MathWorks Inc.
%
% Author: Andrew Rogers, 10 February 2015
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


A = [zeros(3,3) eye(3)
    3*n^2 0 0 0 2*n 0
    0 0 0 -2*n 0 0
    0 0 -n^2 0 0 0];

DX = A*X;
end
