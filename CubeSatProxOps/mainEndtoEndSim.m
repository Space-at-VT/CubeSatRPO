clear,clc
close all

%% Initialization
% Time and orbit parameters
scenario = newScenario;
scenario.T = 15;
scenario.a = 7000e3;

% RSO bounds
RSO = newSatellite(scenario);
RSO.bnd = [0.1,0.2,0.1];

% Deputy parameters
sat = newSatellite(scenario);
sat.EOM = 'LERM';
sat.bnd = [0.1,0.3,0.2];
sat.Tmax = 0.0007/10;
sat.vmax = 0.025;
sat.dryMass = 7;
sat.fuel = 0.5;
sat.kp = 0.01;
sat.kd = 0.025;
sat.wb1 = 5*(pi/180);
sat.wb2 = 5*(pi/180);
sat.wb3 = 10*(pi/180);
sat.point = 1;
sat.pt = [1,0,0];
sat.scenario.dt = 1;

% Cold gas (isobutane)
sat.umax = 0.1;
sat.Isp = 65;

% Deputy initial state
X0 = [10,0,0,1,0,0];

sat.x = X0(1);
sat.y = X0(2);
sat.z = X0(3);
sat.vx = X0(4);
sat.vy = X0(5);
sat.vz = X0(6);
tspan = sat.scenario.TP;

%% Detumble
sat.propagate(tspan);