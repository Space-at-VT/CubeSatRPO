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

% RSO slew
RSO.wb1 = 1*(pi/180);
RSO.wb2 = 0.05*(pi/180);
RSO.wb3 = 0*(pi/180);
RSO.point = -1;

% Deputy parameters
sat = newSatellite(scenario);
sat.EOM = 'LERM';
sat.bnd = [0.3,0.2,0.1];
sat.Tmax = 0.0007;
sat.vmax = 0.1;
sat.dryMass = 7;
sat.fuel = 0.25;
sat.kp = 5e-3;
sat.kd = 5e-3;
sat.pt = [0,0,0];
sat.point = 1;
sat.q3 = 1;
sat.q4 = 0;
tspan = sat.scenario.TP;

% Cold gas (isobutane)
sat.umax = 0.1;
sat.Isp = 65;

% Deputy initial state
X0 = [1,0,0,0,0,0];
sat.x = X0(1);
sat.y = X0(2);
sat.z = X0(3);
sat.vx = X0(4);
sat.vy = X0(5);
sat.vz = X0(6);

% throttle ability
sat.canThrottle = 1;

% Approach
tmax = 3000;
tapp = 1000;
trg = 0.4;

iterapp = length(0:scenario.dt:tapp);
range = linspace(X0(1),trg,iterapp);

ii = 1;
while sat.t(end) < tmax
    R = RSO.Rib;
    
    if sat.t(end) < tapp
        dock = range(ii)*[R(1,1),R(2,1),R(3,1)];
    else
        dock = trg*[R(1,1),R(2,1),R(3,1)];
    end
    
    sat.approach(dock,[],[]);
    RSO.propagate(sat.scenario.dt);
    
    sat.printEphemeris
    ii = ii+1;
end

% Plot pretty data
sat.plotAttitude
sat.plotTrajectory(RSO.lbnd,RSO.ubnd,0.1,1)

save('throttle')

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
    efile = strcat(cd,'\',RSO.name,'.e');
    afile = strcat(cd,'\',RSO.name,'.a');
    root.ExecuteCommand(sprintf('SetState */Satellite/%s FromFile "%s"',RSO.name,efile));
    root.ExecuteCommand(sprintf('SetAttitude */Satellite/%s File "%s"',RSO.name,afile));
    root.ExecuteCommand('VO */Satellite/RSO Pass3D OrbitLead None OrbitTrail None');
%     root.ExecuteCommand(sprintf('SetAttitude */Satellite/%s Profile SunECIZ 90 "1 Jan 2018 00:00:00.00"',RSO.name));
    root.ExecuteCommand('VO */Satellite/RSO Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/cubesat_2u.dae"');
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
%     root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Plus-Y-Plate Deploy -90 0',sat.name));
%     root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Minus-Y-Plate Deploy 90 0',sat.name));

    % View Active satellite and reset animation
    root.ExecuteCommand('VO * ViewFromTo Normal From Satellite/MEV To Satellite/MEV');
    root.ExecuteCommand('VO */Satellite/RSO DynDataText DataDisplay "RIC" Show On PreData "Satellite/MEV" Color yellow');
    root.ExecuteCommand('SetAnimation * AnimationMode xRealTime');
    root.ExecuteCommand('Animate * Reset');

    %% Active Power data prodivders
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
%     % Power analysis
%     root.UnitPreferences.Item('DateFormat').SetCurrentUnit('EpSec');
%     root.ExecuteCommand(sprintf('VO */Satellite/%s SolarPanel Compute "1 Jan 2018 00:00:00" "6 Jan 2018 00:00:00" 60',RSO.name));
%     
%     satObj = scenarioObj.Children.Item(RSO.name);
%     powerDataProvider = satObj.DataProviders.Item('Solar Panel Power');
%     powerDataProviderInterval = powerDataProvider.Exec(scenarioObj.StartTime,scenarioObj.StopTime,1);
%     pTime = cell2mat(powerDataProviderInterval.DataSets.GetDataSetByName('Time').GetValues);
%     pPaneldBW = cell2mat(powerDataProviderInterval.DataSets.GetDataSetByName('Power').GetValues);
%     
%     % Convert dBW to watts
%     pPanel = 10.^(pPaneldBW/10);
%     
%     % Interpolate to Matlab analysis time step
%     time = RSO.t;
%     pPanel = interp1(pTime,pPanel,time,'pchip');
%     
%     % Power draw analysis
%     pLoad = RSO.runPowerAnalysis;
%     
%     % Save data
%     save('PowerData','time','pLoad','pPanel')
%     simpleEMU('PowerData')
end
