%% MEV Precursor Demonstration Concept of Operations
% End to end simulation of the Mission Extension Vehicle Precursor
% Demonstration. Defines and simulates a 6U CubeSat starting at tipoff from
% the launch vehicle. All calculations are done in the LVLH RIC coordinate
% system of a target 3U CubeSat. The target satellite orbits the Earth in a
% noon-midnight sun-synchronous orbit and does not maneuver.
% 
% Simulation begins with an approximation of a detumble phase. The 6U then
% maneuvers to enter a series of relative ellipses, reducing radial
% distance each iteration. Finally, the 6U begins a final docking procedure
% to approach the target satellite travelling in the negative direction on
% the radial axis.
%
% Includes an export of ephemeris files to STK for animation and solar
% power generation from solar panels. Solar panel power generation data is
% then imported back into Matlab and the Energy Management Utility is used
% to estimate the state of charge of the 6U CubeSats batteries.
clear,clc
close all

%% Initialization
% Time and orbit parameters
scenario = newScenario;
scenario.name = 'EndtoEndSimulation';
scenario.a = 7127e3;
scenario.dt = 1;
tspan = scenario.TP;

% Target satellite
RSO = newSatellite(scenario);
RSO.name = 'RSO';
RSO.bnd = [0.3,0.1,0.1];
RSO.dryMass = 4;
RSO.fuel = 0;
RSO.kp = 0.005;
RSO.kd = 0.005;

% Acitve satellite
sat = newSatellite(scenario);
sat.name = 'MEV';
sat.EOM = 'LERM';
sat.bnd = [0.3,0.2,0.1];
sat.Tmax = 0.0007;
sat.vmax = 0.05;
sat.dryMass = 7;
sat.fuel = 0.35;
sat.kp = 0.005;
sat.kd = 0.005;
sat.pt = [0,0,0];

% Cold gas (isobutane)
sat.umax = 0.1;
sat.Isp = 65;

% Active initial state
X0 = [5,0,0,0,0,0.01]';
sat.x = X0(1);
sat.y = X0(2);
sat.z = X0(3);
sat.vx = X0(4);
sat.vy = X0(5);
sat.vz = X0(6);

%% Detumble
fprintf('Detumbling...\n')

% Active
sat.wb1 = 10*(pi/180);
sat.wb2 = -5*(pi/180);
sat.wb3 = 5*(pi/180);
sat.Tmax = 5e-6;
sat.point = 1;
sat.propagate(3*tspan);

% Target
RSO.wb1 = 5*(pi/180);
RSO.wb2 = -3*(pi/180);
RSO.wb3 = -1*(pi/180);
RSO.Tmax = 5e-6;
RSO.point = 0;
RSO.propagate(3*tspan);

%% Absorb momentum
fprintf('Activating reaction wheels...\n')
sat.Tmax = 0.0007;

%% Enter first ellipise
fprintf('Entering first ellipse...\n')
xf = 40;
Xf = [xf,0,0,0,-2*sat.scenario.n*xf,0];
sat.phaseManeuverEq(Xf,tspan,10);
sat.propagate(10*tspan);

%% Enter second ellipise
fprintf('Entering second ellipse...\n')
xf = 20;
Xf = [xf,0,0,0,-2*sat.scenario.n*xf,0];
sat.phaseManeuverEq(Xf,tspan,10);
sat.propagate(10*tspan);

%% Enter third ellipise
fprintf('Entering third ellipse...\n')
xf = 10;
Xf = [xf,0,0,0,-2*sat.scenario.n*xf,0];
sat.phaseManeuverEq(Xf,tspan,10);
sat.propagate(10*tspan);

%% Proximity operations
fprintf('Starting docking approach...\n')
dock = [0.5,0,0];
tol = [0.1,0.1,0.1];
sat.pt = [-100,0,0];

% Approach
sat.scenario.dt = 1;
sat.T = 15;
separation = abs(sat.p-dock);

while separation(1) > tol(1) || separation(2) > tol(2)...
        || separation(3) > tol(3)
    sat.printEphemeris
    sat.approach(dock,[],[]);
    separation = abs(sat.p-dock);
end

% Hold
thold = 0;
tmax = 300;
sat.scenario.dt = 0.5;
sat.T = 15;
tol = [0.2,0.2,0.2];
while thold < tmax
    sat.printEphemeris
    sat.maintain(dock-tol,dock+tol);
    thold = thold+sat.scenario.dt;
end

%% Conjoined flight
sat.fixAttitude([0,0,1,0])
sat.fixPosition([0,0,0])
sat.scenario.dt = 60;
sat.propagate(10*tspan);

%% Propagate Target
RSO.fixAttitude([0,0,0,1])
RSO.scenario.dt = 60;
RSO.propagate(sat.t(end)-3*tspan);

%% Plot pretty data
% Active
sat.plotAttitude
sat.plotFuel
sat.plotTrajectory(RSO.lbnd,RSO.ubnd,1,1)
RSO.plotAttitude

%% STK Analysis
% Create scenario
[app,root] = scenario.createSTKscenario;

% Create satellites
RSO.createSTKobject(app,root,'3U')
sat.createSTKobject(app,root,'6U')

% Run power analysis
RSO.powerAnalysis('3U');
sat.powerAnalysis('6U');
