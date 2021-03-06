clear,clc
close all

STK = 0;

scenario = newScenario;
scenario.tmax = 600;
scenario.T = 60;
scenario.dt = 2;

sat = newSatellite;
sat.EOM = 'LERM';
sat.bnd = [0.3,0.3,0.3];
sat.x = 5;
sat.y = 0;
sat.z = 0;
sat.vmax = 0.05;
sat.umax = 0.02;
sat.dryMass = 8;
sat.fuel = 1.167;
sat.ISP = 65;

chief = newSatellite;
chief.name = 'Shuttle';

p = [-5,10,7];

shuttleUbnd = [12,-5,1
               4,20,4
               -3,5,15];
shuttleLbnd = [-5,-16,-1
               -5,-17,-4
               -5,-9,-15];
          
while scenario.t <= scenario.tmax
    clc
    fprintf('Time: %5.1f\n',scenario.t)
    
    if separation(sat.p,p) < 0.25
        break
    end
    
    chief = chief.approach(scenario,chief.p);
    sat = sat.approach(scenario,p,shuttleLbnd,shuttleUbnd);
        
    clf
    for ii = 1:size(shuttleLbnd,1)
        plotObstacle(shuttleLbnd(ii,:),shuttleUbnd(ii,:),'-k');
    end
    plotShuttle(0,0,0,0,0,0,0.05,1e-3,[1,1,0.5])
    plotTrajectory(sat);
    pause(1e-10)

    scenario.t = scenario.t+scenario.dt;
end
plotControls(sat,scenario)
save('ShuttleInspection')

if STK
    createSTKfile(chief,scenario);
    
    sat.name = 'Inspection';
    createSTKfile(sat,scenario);
    
    app = actxserver('STK11.application');
    root = app.Personality2;
    
    root.ExecuteCommand('New / Scenario ShuttleInspection');
    
    root.ExecuteCommand('New / */Satellite Origin');
    root.ExecuteCommand(sprintf('SetState */Satellite/Origin Classical TwoBody UseScenarioInterval 60 ICRF "1 Jan 2000" %f 0 28.5 0 0 0',scenario.a));
    root.ExecuteCommand('VO * ObjectStateInWin Show off Object Satellite/Origin WindowId 1');
    
    root.ExecuteCommand(sprintf('New / */Satellite Chief'));
    file = strcat(cd,'\Shuttle.e');
    root.ExecuteCommand(sprintf('SetState */Satellite/Chief FromFile "%s"',file));
    root.ExecuteCommand('VO */Satellite/Chief Pass3D OrbitLead None OrbitTrail None');
    root.ExecuteCommand('VO */Satellite/Chief Model File "C:\Program Files\AGI\STK 11\STKData\VO\Models\Space\shuttle-05.mdl"');
    root.ExecuteCommand('VO */Satellite/Chief Articulate "1 Jan 2000" 0 Shuttle_05 Size 1 0.75');
    
    for ii = 1:length(sat)
        root.ExecuteCommand(sprintf('New / */Satellite %s',sat(ii).name));
        file = strcat(cd,'\',sat(ii).name,'.e');
        root.ExecuteCommand(sprintf('SetState */Satellite/%s FromFile "%s"',sat(ii).name,file));
        root.ExecuteCommand(sprintf('VO */Satellite/%s Pass3D OrbitLead None OrbitTrail None',sat(ii).name));
        root.ExecuteCommand(sprintf('VO */Satellite/%s Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/cubesat.mdl"',sat(ii).name));
    end
    
    root.ExecuteCommand('SetAnimation * AnimationMode xRealTime');
    root.ExecuteCommand('Animate * Reset');
end