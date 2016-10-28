 clear,clc
close all

% Time and orbit parameters
scenario = newScenario;
scenario.T = 15;
scenario.a = 7000e3;

% Chief bounds
chief = newSatellite;
chief.bnd = [0.1,0.2,0.1];

% Deputy parameters
deputy = newSatellite;
deputy.EOM = 'LERM';
deputy.bnd = [0.1,0.3,0.2];
deputy.umax = 0.1;
deputy.Tmax = 0.007;
deputy.vmax = 0.1;
deputy.dryMass = 11.5;
deputy.fuel = 0.5;
deputy.kp = 0.1;
deputy.kd = 0.1;
deputy.point = 1;

% Deputy initial state
deputy.x = 200;
deputy.y = -100;
deputy.z = 100;
tspan = scenario.TP;

Xf = [10,0,0,0,-2*scenario.n*10,0];
deputy.phaseManeuver(scenario,Xf,tspan,10);
deputy.propagate(scenario,tspan);

chief.propagate(scenario,4*tspan); 

dock = [1,0,0];
while true
    clc
    fprintf('Time: %5.1f\n',deputy.t(end)) 
    
    deputy.approach(scenario,dock,chief.lbnd,chief.ubnd);
    if separation(deputy.p,dock) < 0.01,break,end

    clf
    deputy.plotTrajectory(chief.lbnd,chief.ubnd,3);
end

dock = [0.25,0,0];
while true
    clc
    fprintf('Time: %5.1f\n',deputy.t(end)) 
    
    deputy.approach(scenario,dock,chief.lbnd,chief.ubnd);
    if separation(deputy.p,dock) < 0.01,break,end

    clf
    deputy.plotTrajectory(chief.lbnd,chief.ubnd,3);
end

deputy.plotControls;
 
STK = 1;
if STK
    deputy.name = 'MEV';
    chief.name = 'RSO';
    createSTKfile(deputy,scenario);
    createSTKfile(chief,scenario);
    
    app = actxserver('STK11.application');
    root = app.Personality2;
    
    root.ExecuteCommand('New / Scenario CubeSat');
    root.ExecuteCommand('SetAnalysisTimePeriod * "9 Oct 2016 16:00:00" "10 Oct 2016 00:00:00"');
    
    root.ExecuteCommand('New / */Satellite Origin');
    root.ExecuteCommand(sprintf('SetState */Satellite/Origin Classical TwoBody UseScenarioInterval 60 ICRF "1 Jan 2000" %f 0 28.5 0 0 0',scenario.a));
    root.ExecuteCommand('VO * ObjectStateInWin Show off Object Satellite/Origin WindowId 1');
    
    root.ExecuteCommand(sprintf('New / */Satellite Chief'));
    file = strcat(cd,'\',chief.name,'.e');
    root.ExecuteCommand(sprintf('SetState */Satellite/Chief FromFile "%s"',file));
    root.ExecuteCommand('VO */Satellite/Chief Pass3D OrbitLead None OrbitTrail None');
    root.ExecuteCommand('VO */Satellite/Chief Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/cubesat_2u.dae"');  
    
    root.ExecuteCommand(sprintf('New / */Satellite %s',deputy.name));
    efile = strcat(cd,'\',deputy.name,'.e');
    afile = strcat(cd,'\',deputy.name,'.a');
    root.ExecuteCommand(sprintf('SetState */Satellite/%s FromFile "%s"',deputy.name,efile));
    root.ExecuteCommand(sprintf('SetAttitude */Satellite/%s File "%s"',deputy.name,afile));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Pass3D OrbitLead None OrbitTrail None',deputy.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/cubesat_6u.dae"',deputy.name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 6U-Cubesat Yaw 0 180',deputy.name));
    
    root.ExecuteCommand('VO * ViewFromTo Normal From Satellite/MEV To Satellite/MEV');
    root.ExecuteCommand('SetAnimation * AnimationMode xRealTime');
    root.ExecuteCommand('Animate * Reset');
end
