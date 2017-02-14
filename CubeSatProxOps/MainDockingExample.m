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
sat.Tmax = 0.0007;
sat.vmax = 0.025;
sat.dryMass = 7;
sat.fuel = 0.5;
sat.kp = 0.005;
sat.kd = 0.025;
sat.point = 1;
sat.pt = [0,0,0];
sat.scenario.dt = 5;

% Low thrust monoprop (hydrazine)
% sat.umax = 0.1;
% sat.Isp = 200;

% High thrust monoprop (green)
% sat.umax = 0.25;
% sat.Isp = 250;

% Cold gas (isobutane)
sat.umax = 0.1;
sat.Isp = 65;

% Deputy initial state
X0 = [84.285,14066.592,15.218,1.764271,-0.388140,0.074808];
% X0 = [84.285,66.592,0,1,0,0];
% X0 = [84.285,66.592,15.218,0.5,0,0];

% sat.x = 200;
% sat.y = -100;
% sat.z = 100;
sat.x = X0(1);
sat.y = X0(2);
sat.z = X0(3);
sat.vx = X0(4);
sat.vy = X0(5);
sat.vz = X0(6);
tspan = sat.scenario.TP;

%% Rendezvous
% First relative ellipse
X0 = 100;
Xf = [X0,0,0,0,-2*sat.scenario.n*X0,0];
sat.phaseManeuverEq(Xf,tspan,5);
sat.propagate(5*tspan);

% Second relative ellipse
X0 = 10;
Xf = [X0,0,0,0,-2*sat.scenario.n*X0,0];
sat.phaseManeuverEq(Xf,tspan,5);
sat.propagate(5*tspan);

%% Proximity operations
% dock = [0.4,0,0];
% tol = [0.025,0.025,0.025];
% 
% % Approach
% while separation(sat.p,dock,1) > tol(1) || separation(sat.p,dock,2) > tol(2)...
%         || separation(sat.p,dock,3) > tol(3)
%     sat.printEphemeris
%     sat.approach(dock,RSO.lbnd,RSO.ubnd);
% end
% 
% % Hold  
% thold = 0;
% tmax = 600;
% while thold < tmax
%     sat.maintain(dock-tol,dock+tol);
%     thold = thold+sat.scenario.dt;
% end
% 
% RSO.propagate(sat.t(end));

%% Post process
% close all
% sat.plotControls;
% sat.subplotTrajectory;
pLoad = sat.runPowerAnalysis('LoadData');
% sat.renderVideo('movie1.avi',RSO.lbnd,RSO.ubnd,25);

%% STK Analysis
STK = 1;
if STK
    % Export to STK
    sat.name = 'MEV';
    RSO.name = 'RSO';
    createSTKfile(sat,scenario);
    createSTKfile(RSO,scenario);
    
    % Open STK
    app = actxserver('STK11.application');
    root = app.Personality2;
    
    % Create scenario
    root.ExecuteCommand('New / Scenario MainDockingExample');
    root.ExecuteCommand('SetAnalysisTimePeriod * "1 Jan 2018 00:00:00" "1 Feb 2018 00:00:00"');
    
    % Create origin
    root.ExecuteCommand('New / */Satellite Origin');
    root.ExecuteCommand(sprintf('SetState */Satellite/Origin Classical TwoBody UseScenarioInterval 60 ICRF "1 Jan 2000" %f 0 28.5 0 0 0',scenario.a));
    root.ExecuteCommand('VO * ObjectStateInWin Show off Object Satellite/Origin WindowId 1');
    
    % Create Target satellite
    root.ExecuteCommand(sprintf('New / */Satellite RSO'));
    file = strcat(cd,'\',RSO.name,'.e');
    root.ExecuteCommand(sprintf('SetState */Satellite/RSO FromFile "%s"',file));
    root.ExecuteCommand('VO */Satellite/RSO Pass3D OrbitLead None OrbitTrail None');
    root.ExecuteCommand('VO */Satellite/RSO Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/cubesat_2u.dae"');  
    
    % Create Active satellite
    root.ExecuteCommand(sprintf('New / */Satellite %s',sat.name));
    efile = strcat(cd,'\',sat.name,'.e');
    afile = strcat(cd,'\',sat.name,'.a');
    root.ExecuteCommand(sprintf('SetState */Satellite/%s FromFile "%s"',sat.name,efile));
    root.ExecuteCommand(sprintf('SetAttitude */Satellite/%s File "%s"',sat.name,afile));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Pass3D OrbitLead None OrbitTrail None',sat.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/cubesat_6u.dae"',sat.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 6U-Cubesat Yaw 0 180',sat.name));
    
    % View Active satellite and reset animation
    root.ExecuteCommand('VO * ViewFromTo Normal From Satellite/MEV To Satellite/MEV');
    root.ExecuteCommand('VO */Satellite/RSO DynDataText DataDisplay "RIC" Show On PreData "Satellite/MEV" Color yellow');
    root.ExecuteCommand('SetAnimation * AnimationMode xRealTime');
    root.ExecuteCommand('Animate * Reset');
    
    % Power analysis
    root.UnitPreferences.Item('DateFormat').SetCurrentUnit('EpSec');
    root.ExecuteCommand(sprintf('VO */Satellite/%s SolarPanel Compute "1 Jan 2018 00:00:00" "2 Jan 2018 00:00:00" 60',sat.name));
    
    % Power data prodivders
    scenarioObj = root.CurrentScenario;
    satObj = scenarioObj.Children.Item(sat.name);
    powerDataProvider = satObj.DataProviders.Item('Solar Panel Power');
    powerDataProviderInterval = powerDataProvider.Exec(scenarioObj.StartTime,scenarioObj.StopTime,1);
    pTime = cell2mat(powerDataProviderInterval.DataSets.GetDataSetByName('Time').GetValues);
    pPaneldBW = cell2mat(powerDataProviderInterval.DataSets.GetDataSetByName('Power').GetValues);
    
    % Convert dBW to watts
    pPanel = 10.^(pPaneldBW/10);
    
    % Interpolate to Matlab analysis time step
    pPanel = interp1(pTime,pPanel,sat.t,'pchip');
    
    % Save data
    time = sat.t;
    save('PowerData','time','pLoad','pPanel')    
end
