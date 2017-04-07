clear,clc
close all

%% Initialization
% Time and orbit parameters
scenario = newScenario;
scenario.a = 42164e3;
scenario.dt = 1;

% RSO bounds
RSO = newSatellite(scenario);
RSO.bnd = [1,1,1];
RSO.dryMass = 1000;

% RSO slew
RSO.wb1 = 0*(pi/180);
RSO.wb2 = 0.25*(pi/180);
RSO.wb3 = 0.25*(pi/180);
RSO.point = -1;

% Deputy parameters
sat = newSatellite(scenario);
sat.EOM = 'LERM';
sat.bnd = [0.5,0.5,0.5];
sat.Tmax = 0.1;
sat.vmax = 0.1;
sat.dryMass = 50;
sat.fuel = 5;
sat.kp = 9e-2;
sat.kd = 5e-1;
sat.pt = [0,0,0];
sat.point = 1;
sat.q3 = 1;
sat.q4 = 0;
tspan = sat.scenario.TP;

% Hydrazine
sat.umax = 1;
sat.Isp = 200;

% Deputy initial state
X0 = [10,0,0,0,0,0];
sat.x = X0(1);
sat.y = X0(2);
sat.z = X0(3);
sat.vx = X0(4);
sat.vy = X0(5);
sat.vz = X0(6);

% Approach
sat.T = 15;
tmax = 2400;

trg = 5;
thalf = round(length(0:scenario.dt:tmax)/2);
range = [linspace(X0(1),trg,thalf),trg*ones(1,thalf)];

ii = 1;
while sat.t(end) < tmax
    R = RSO.Rib;
    dock = range(ii)*[R(1,1),R(2,1),R(3,1)];
    
    sat.approach(dock,[],[]);
    RSO.propagate(sat.scenario.dt);
    
    sat.printEphemeris
    ii = ii+1;
end

% Plot pretty data
sat.plotAttitude
sat.plotFuel
sat.plotTrajectory(RSO.lbnd,RSO.ubnd,0.1,1)

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
    root.ExecuteCommand(sprintf('SetState */Satellite/Origin Classical TwoBody UseScenarioInterval 60 ICRF "1 Jan 2018" %f 0 0 0 0 270',scenario.a));
    root.ExecuteCommand('VO * ObjectStateInWin Show off Object Satellite/Origin WindowId 1');
    
    % Create Target satellite
    root.ExecuteCommand(sprintf('New / */Satellite RSO'));
    efile = strcat(cd,'\',RSO.name,'.e');
    afile = strcat(cd,'\',RSO.name,'.a');
    root.ExecuteCommand(sprintf('SetState */Satellite/%s FromFile "%s"',RSO.name,efile));
    root.ExecuteCommand(sprintf('SetAttitude */Satellite/%s File "%s"',RSO.name,afile));
    root.ExecuteCommand('VO */Satellite/RSO Pass3D OrbitLead None OrbitTrail None');
    root.ExecuteCommand('VO */Satellite/RSO Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/fs1300.mdl"');
    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Attitude yrot 0 270',RSO.name));
    
    % Create Active satellite
    root.ExecuteCommand(sprintf('New / */Satellite %s',sat.name));
    efile = strcat(cd,'\',sat.name,'.e');
    afile = strcat(cd,'\',sat.name,'.a');
    root.ExecuteCommand(sprintf('SetState */Satellite/%s FromFile "%s"',sat.name,efile));
    root.ExecuteCommand(sprintf('SetAttitude */Satellite/%s File "%s"',sat.name,afile));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Pass3D OrbitLead None OrbitTrail None',sat.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Satellite Roll 0 90',sat.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Satellite Yaw 0 90',sat.name));
    
    % View Active satellite and reset animation
    root.ExecuteCommand('VO * ViewFromTo Normal From Satellite/MEV To Satellite/MEV');
    root.ExecuteCommand('VO */Satellite/RSO DynDataText DataDisplay "RIC" Show On PreData "Satellite/MEV" Color yellow');
    root.ExecuteCommand('SetAnimation * AnimationMode xRealTime');
    root.ExecuteCommand('Animate * Reset');


end
