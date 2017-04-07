clear,clc
close all

%% Initialization
% Time and orbit parameters
scenario = newScenario;
scenario.T = 15;
scenario.a = 6803.1366e3;
scenario.ecc = 0.5;

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
sat.scenario.dt = 0.5;

% Cold gas (isobutane)
sat.umax = 0.25;
sat.Isp = Inf;

% Eccentric factor
eccF = -sat.scenario.n*(2+scenario.ecc)/...
    (sqrt((1+scenario.ecc)*(1-scenario.ecc)^3));
vz = 0.5;

% Deputy initial state
X = 200;
X0 = [X,0,0,0,eccF*X,0.5];
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
X0 = 400;
Xf = [X0,0,0,0,eccF*X0,0.5];
sat.scenario.dt = 5;
sat.phaseManeuverEq(Xf,tspan,5);

% Propagate
sat.scenario.dt = 0.5;
sat.propagate(tspan);

% Plot
sat.subplotTrajectory
sat.plotControls

format long
sat.dv(end)
