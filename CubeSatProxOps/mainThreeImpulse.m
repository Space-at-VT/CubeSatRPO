clear,clc
close all

%% Initialization
% Time and orbit parameters
scenario = newScenario;
scenario.T = 15;
scenario.a = 6803.1366e3;

% RSO bounds
RSO = newSatellite(scenario);
RSO.bnd = [0.1,0.2,0.1];

% Deputy parameters
sat = newSatellite(scenario);
sat.EOM = 'LERM';
sat.bnd = [0.1,0.3,0.2];
sat.Tmax = 0.0007;
sat.vmax = 0.025;
sat.dryMass = 6.5;
sat.fuel = 0.5;
sat.kp = 0.005;
sat.kd = 0.025;
sat.point = 1;
sat.pt = [0,0,0];
sat.scenario.dt = 1;

% Cold gas (isobutane)
% sat.umax = 0.025;
sat.umax = 5;
sat.Isp = inf;

% Deputy initial state
X = 100;
X0 = [X,0,0,0,-2*sat.scenario.n*X,0];
sat.x = X0(1);
sat.y = X0(2);
sat.z = X0(3);
sat.vx = X0(4);
sat.vy = X0(5);
sat.vz = X0(6);
tspan = sat.scenario.TP;

%% Rendezvous
% Propagate one orbit
sat.propagate(tspan);

% Maneuver
X0 = 50;
Xf = [X0,0,0,0,-2*sat.scenario.n*X0,0];
sat.phaseManeuverEq(Xf,tspan,5);

% Propagate
sat.propagate(tspan);

sat.dv(end)

% Plot
sat.plotControls
sat.plotAttitude
sat.plotTrajectory