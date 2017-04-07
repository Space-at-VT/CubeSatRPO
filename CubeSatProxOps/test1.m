clear,clc
% close all

% Time and orbit parameters
scenario = newScenario;
scenario.a = 6878e3;

% Deputy parameters
sat = newSatellite(scenario);
sat.umax = 10;

% Deputy initial state

X0 = [250,300,100,0.8814,-0.5316,0.3499];
sat.x = X0(1);
sat.y = X0(2);
sat.z = X0(3);
sat.vx = X0(4);
sat.vy = X0(5);
sat.vz = X0(6);

X0 = 0;
Xf = [0,0,0,0,0,0];
tspan = 1.5*3600;
sat.phaseManeuverEq(Xf,tspan,10);
% sat.propagate(tspan);

sat.plotTrajectory([],[],1,1)
sat.dv(end)