clear,clc
close all

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
sat.d = 0.02*sat.bnd;
sat.umax = 0.1;
sat.Tmax = 0.007;
sat.vmax = 0.05;
sat.dryMass = 11.5;
sat.fuel = 0.5;
sat.kp = 0.1;
sat.kd = 0.1;
sat.point = 1;
sat.pt = [0,0,0];

% Deputy initial state
sat.x = 600;
sat.y = -100;
sat.z = 100;
tspan = sat.scenario.TP;

% First relative ellipse
Xf = [40,0,0,0,-2*sat.scenario.n*40,0.05];
sat.phaseManeuver3(Xf,tspan,30);
sat.propagate(tspan);

% Second relative ellipse
Xf = [5,0,0,0,-2*sat.scenario.n*5,0];
sat.phaseManeuver3(Xf,tspan,10);
sat.propagate(tspan);
sat.plotTrajectory(RSO.lbnd,RSO.ubnd,3);

% Proximity operations
dock = [0.5,0,0];
thold = 0;
tmax = 30;
while true
    clc,fprintf('Time: %5.1f\n',sat.t(end))
    
    if separation(sat.p,dock) < 0.1
        sat.maintain(dock-0.1,dock+0.1);
        thold = thold+sat.scenario.dt;
    else
        sat.approach(dock,RSO.lbnd,RSO.ubnd);
        thold = 0;
    end
    
    if thold > tmax,break,end
    
    clf,sat.plotTrajectory(RSO.lbnd,RSO.ubnd,3);
end
sat.plotControls;
RSO.propagate(sat.t(end));

% Create movie
rec = 0;
if rec
    sat.renderVideo('movie1.avi',RSO.lbnd,RSO.ubnd,25);
end

% Export to STK
STK = 1;
if STK
    pause(0.1)
    sat.name = 'MEV';
    RSO.name = 'RSO';
    createSTKfile(sat,scenario);
    createSTKfile(RSO,scenario);
     
    app = actxserver('STK11.application');
    root = app.Personality2;
    
    root.ExecuteCommand('New / Scenario CubeSat');
    root.ExecuteCommand('SetAnalysisTimePeriod * "9 Oct 2016 16:00:00" "10 Oct 2016 00:00:00"');
    
    root.ExecuteCommand('New / */Satellite Origin');
    root.ExecuteCommand(sprintf('SetState */Satellite/Origin Classical TwoBody UseScenarioInterval 60 ICRF "1 Jan 2000" %f 0 28.5 0 0 0',scenario.a));
    root.ExecuteCommand('VO * ObjectStateInWin Show off Object Satellite/Origin WindowId 1');
    
    root.ExecuteCommand(sprintf('New / */Satellite RSO'));
    file = strcat(cd,'\',RSO.name,'.e');
    root.ExecuteCommand(sprintf('SetState */Satellite/RSO FromFile "%s"',file));
    root.ExecuteCommand('VO */Satellite/RSO Pass3D OrbitLead None OrbitTrail None');
    root.ExecuteCommand('VO */Satellite/RSO Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/cubesat_2u.dae"');  
    
    root.ExecuteCommand(sprintf('New / */Satellite %s',sat.name));
    efile = strcat(cd,'\',sat.name,'.e');
    afile = strcat(cd,'\',sat.name,'.a');
    root.ExecuteCommand(sprintf('SetState */Satellite/%s FromFile "%s"',sat.name,efile));
    root.ExecuteCommand(sprintf('SetAttitude */Satellite/%s File "%s"',sat.name,afile));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Pass3D OrbitLead None OrbitTrail None',sat.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/cubesat_6u.dae"',sat.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 6U-Cubesat Yaw 0 180',sat.name));
    
    root.ExecuteCommand('VO * ViewFromTo Normal From Satellite/MEV To Satellite/MEV');
    root.ExecuteCommand('VO */Satellite/RSO DynDataText DataDisplay "RIC" Show On PreData "Satellite/MEV" Color yellow');
    root.ExecuteCommand('SetAnimation * AnimationMode xRealTime');
    root.ExecuteCommand('Animate * Reset');
end
