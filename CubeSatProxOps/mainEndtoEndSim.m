%% MEV Precursor Demonstration Concept of Operations
% End to end simulation of the Mission Extension Vehhicle Precursor
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
scenario.a = 7127e3;
scenario.dt = 1;

% RSO bounds
RSO = newSatellite(scenario);
RSO.bnd = [0.3,0.1,0.1];
RSO.dryMass = 4;
RSO.fuel = 0;

% Deputy parameters
sat = newSatellite(scenario);
sat.EOM = 'LERM';
sat.bnd = [0.3,0.2,0.1];
sat.Tmax = 0.0007;
sat.vmax = 0.025;
sat.dryMass = 7;
sat.fuel = 0.25;
sat.kp = 0.005;
sat.kd = 0.005;
sat.pt = [0,0,0];
tspan = scenario.TP;

% Cold gas (isobutane)
sat.umax = 0.1;
sat.Isp = 65;

% Deputy initial state
X0 = [5,0,0,0,0,0.01];
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
sat.Tmax = 0.000005;
sat.point = 1;
sat.propagate(3*tspan);

% Target
RSO.wb1 = 5*(pi/180);
RSO.wb2 = -3*(pi/180);
RSO.wb3 = 3*(pi/180);
RSO.Tmax = 0.000005;
RSO.point = 0;
RSO.propagate(3*tspan);

%% Absorb momentum
fprintf('Activating reaction wheels...\n')
sat.Tmax = 0.0007;

%% Enter first ellipise
fprintf('Entering first ellipse...\n')
X0 = 40;
Xf = [X0,0,0,0,-2*sat.scenario.n*X0,0];
sat.phaseManeuverEq(Xf,tspan,10);
sat.propagate(10*tspan);

%% Enter second ellipise
fprintf('Entering second ellipse...\n')
X0 = 20;
Xf = [X0,0,0,0,-2*sat.scenario.n*X0,0];
sat.phaseManeuverEq(Xf,tspan,10);
sat.propagate(10*tspan);

%% Enter third ellipise
fprintf('Entering third ellipse...\n')
X0 = 10;
Xf = [X0,0,0,0,-2*sat.scenario.n*X0,0];
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
sat.vmax = 0.05;
while separation(sat.p,dock,1) > tol(1) || separation(sat.p,dock,2) > tol(2)...
        || separation(sat.p,dock,3) > tol(3)
    sat.printEphemeris
%     sat.approach(dock,RSO.lbnd,RSO.ubnd);
    sat.approach(dock,[],[]);
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
sat.x(end) = 0;
sat.y(end) = 0;
sat.z(end) = 0;
sat.vx(end) = 0;
sat.vy(end) = 0;
sat.vz(end) = 0;

sat.scenario.dt = 1;
sat.propagate(10*tspan);

%% Plot pretty data
sat.plotAttitude
sat.plotFuel
sat.plotTrajectory(RSO.lbnd,RSO.ubnd,1,1)

%% Propagate RSO
RSO.scenario.dt = 60;
RSO.propagate(sat.t(end)-3*tspan);

%% STK Analysis
STK = 1;
if STK
    % Export to STK
    sat.name = 'MEV';
    RSO.name = 'RSO';
    createSTKfile(sat);
    createSTKfile(RSO);
    
    % Open STK
    app = actxserver('STK11.application');
    root = app.Personality2;
    
    % Create scenario
    root.ExecuteCommand('New / Scenario MainDockingExample');
    root.ExecuteCommand('SetAnalysisTimePeriod * "1 Jan 2018 00:00:00" "1 Feb 2018 00:00:00"');
    scenarioObj = root.CurrentScenario;
    
    % Create origin
    root.ExecuteCommand('New / */Satellite Origin');
    root.ExecuteCommand(sprintf('SetState */Satellite/Origin Classical TwoBody UseScenarioInterval 60 ICRF "1 Jan 2018" %f 0 98.45 0 90 180',scenario.a));
    root.ExecuteCommand('VO * ObjectStateInWin Show off Object Satellite/Origin WindowId 1');
    
    % Create Target satellite
    root.ExecuteCommand(sprintf('New / */Satellite RSO'));
    file = strcat(cd,'\',RSO.name,'.e');
    root.ExecuteCommand(sprintf('SetState */Satellite/RSO FromFile "%s"',file));
    root.ExecuteCommand('VO */Satellite/RSO Pass3D OrbitLead None OrbitTrail None');
%     root.ExecuteCommand(sprintf('SetAttitude */Satellite/%s Profile SunECIZ 90 "1 Jan 2018 00:00:00.00"',RSO.name));
    root.ExecuteCommand('VO */Satellite/RSO Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/cubesat_2u.dae"');
%     root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Cubesat_2U Pitch 0 90',RSO.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Plus-Z-Panel Deploy -120 0',RSO.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Minus-Z-Panel Deploy 120 0',RSO.name));
    
    % Keep two deployables
%     root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Plus-Y-Panel Deploy 120 0',RSO.name));
%     root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Minus-Y-Panel Deploy -120 0',RSO.name));
    
    % Create Active satellite
    root.ExecuteCommand(sprintf('New / */Satellite %s',sat.name));
    efile = strcat(cd,'\',sat.name,'.e');
    afile = strcat(cd,'\',sat.name,'.a');
    root.ExecuteCommand(sprintf('SetState */Satellite/%s FromFile "%s"',sat.name,efile));
    root.ExecuteCommand(sprintf('SetAttitude */Satellite/%s File "%s"',sat.name,afile));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Pass3D OrbitLead None OrbitTrail None',sat.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/cubesat_6u.dae"',sat.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 6U-Cubesat Yaw 0 180',sat.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Plus-Z-Plate Deploy 90 0',sat.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Minus-Z-Plate Deploy -90 0',sat.name));
    
    % Turn off Deployables
    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Plus-Y-Plate Deploy -90 0',sat.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Minus-Y-Plate Deploy 90 0',sat.name));

    % View Active satellite and reset animation
    root.ExecuteCommand('VO * ViewFromTo Normal From Satellite/MEV To Satellite/MEV');
    root.ExecuteCommand('VO */Satellite/RSO DynDataText DataDisplay "RIC" Show On PreData "Satellite/MEV" Color yellow');
    root.ExecuteCommand('SetAnimation * AnimationMode xRealTime');
    root.ExecuteCommand('Animate * Reset');

    %% ACtive Power data prodivders
    % Power analysis
%     root.UnitPreferences.Item('DateFormat').SetCurrentUnit('EpSec');
%     root.ExecuteCommand(sprintf('VO */Satellite/%s SolarPanel Compute "1 Jan 2018 00:00:00" "6 Jan 2018 00:00:00" 60',sat.name));
%     
%     satObj = scenarioObj.Children.Item(sat.name);
%     powerDataProvider = satObj.DataProviders.Item('Solar Panel Power');
%     powerDataProviderInterval = powerDataProvider.Exec(scenarioObj.StartTime,scenarioObj.StopTime,1);
%     pTime = cell2mat(powerDataProviderInterval.DataSets.GetDataSetByName('Time').GetValues);
%     pPaneldBW = cell2mat(powerDataProviderInterval.DataSets.GetDataSetByName('Power').GetValues);
%     
%     % Convert dBW to watts
%     pPanel = 10.^(pPaneldBW/10);
%     
%     % Interpolate to Matlab analysis time step
%     pPanel = interp1(pTime,pPanel,sat.t,'pchip');
%     
%     % Power draw analysis
%     pLoad = sat.runPowerAnalysis;
%     
%     % Save data
%     time = sat.t;
%     save('PowerData','time','pLoad','pPanel')
%     simpleEMU('PowerData')
    
    %% Target Power Data
    % Power analysis
    root.UnitPreferences.Item('DateFormat').SetCurrentUnit('EpSec');
    root.ExecuteCommand(sprintf('VO */Satellite/%s SolarPanel Compute "1 Jan 2018 00:00:00" "6 Jan 2018 00:00:00" 60',RSO.name));
    
    satObj = scenarioObj.Children.Item(RSO.name);
    powerDataProvider = satObj.DataProviders.Item('Solar Panel Power');
    powerDataProviderInterval = powerDataProvider.Exec(scenarioObj.StartTime,scenarioObj.StopTime,1);
    pTime = cell2mat(powerDataProviderInterval.DataSets.GetDataSetByName('Time').GetValues);
    pPaneldBW = cell2mat(powerDataProviderInterval.DataSets.GetDataSetByName('Power').GetValues);
    
    % Convert dBW to watts
    pPanel = 10.^(pPaneldBW/10);
    
    % Interpolate to Matlab analysis time step
    time = RSO.t;
    pPanel = interp1(pTime,pPanel,time,'pchip');
    
    % Power draw analysis
    pLoad = RSO.runPowerAnalysis;
    
    % Save data
    save('PowerData','time','pLoad','pPanel')
    simpleEMU('PowerData')
end
