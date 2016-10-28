clear,clc
close all

STK = 1;

scenario = newScenario;
scenario.tmax = 60;
scenario.T = 15;
scenario.dt = 1;

sat = newSatellite;
sat.EOM = 'LERM';

sat.bnd = [0.1,0.3,0.2];
% sat.d = [0.001,0.003,0.002];
sat.x = 5;
sat.y = 0;
sat.z = 0;
sat.vmax = 0.5;
sat.umax = 0.25;
sat.Tmax = 0.007;
sat.dryMass = 11.5;
sat.fuel = 0.5;
sat.kp = 0.1;
sat.kd = 0.1;
sat.point = 1;
sat.pt = [0,0,0];

chief = newSatellite;
chief.name = 'Shuttle';

% p(1,:) = [-5,-5,18];
% p(2,:) = [-6,-5,0];
% p(3,:) = [-5,-5,-18];
% p(4,:) = sat.p;
p(1,:) = [-7,-12,6];
p(2,:) = [-4,-20,0];
p(3,:) = [0,-12,-6];
p(4,:) = sat.p;

shuttleUbnd = [12,-5,1
               4,20,4
               -3,5,15];
shuttleLbnd = [-5,-16,-1
               -5,-17,-4
               -5,-9,-15];
           
iter = 1;
loiter = 30;
delay = 0;
while scenario.t <= scenario.tmax
    clc
    fprintf('Time: %5.1f\n',scenario.t)

    if separation(sat.p,p(iter,:)) < 0.25
        delay = delay+1;
        if delay == loiter
            iter = iter+1;
            if iter > size(p,1),iter = 1;end
            delay = 0;
        end
    end
    
    chief.approach(scenario,chief.p);
    sat.approach(scenario,p(iter,:),shuttleLbnd,shuttleUbnd);
    
    clf
    plotShuttle(0,0,0,0,0,0,0.05,1e-3,[1,1,0.5])
    sat.plotTrajectory(shuttleLbnd,shuttleUbnd,5);

    scenario.t = scenario.t+scenario.dt;
end
sat.plotControls
save('ShuttleInspection')

if STK
    createSTKfile(chief,scenario);
    
    sat.name = 'Inspection';
    createSTKfile(sat,scenario);
    
    app = actxserver('STK11.application');
    root = app.Personality2;
    
    root.ExecuteCommand('New / Scenario ShuttleInspection');
    root.ExecuteCommand('SetAnalysisTimePeriod * "9 Oct 2016 16:00:00" "10 Oct 2016 00:00:00"');
    
    root.ExecuteCommand('New / */Satellite Origin');
    root.ExecuteCommand(sprintf('SetState */Satellite/Origin Classical TwoBody UseScenarioInterval 60 ICRF "1 Jan 2000" %f 0 28.5 0 0 180',scenario.a));
    root.ExecuteCommand('VO * ObjectStateInWin Show off Object Satellite/Origin WindowId 1');
    
    root.ExecuteCommand(sprintf('New / */Satellite Chief'));
    file = strcat(cd,'\Shuttle.e');
    root.ExecuteCommand(sprintf('SetState */Satellite/Chief FromFile "%s"',file));
    root.ExecuteCommand('VO */Satellite/Chief Pass3D OrbitLead None OrbitTrail None');
    root.ExecuteCommand('VO */Satellite/Chief Model File "C:\Program Files\AGI\STK 11\STKData\VO\Models\Space\shuttle-05.mdl"');
    root.ExecuteCommand('VO */Satellite/Chief Articulate "1 Jan 2000" 0 Shuttle_05 Size 1 0.75');    
    
    for ii = 1:length(sat)
        root.ExecuteCommand(sprintf('New / */Satellite %s',sat(ii).name));
        efile = strcat(cd,'\',sat(ii).name,'.e');
        afile = strcat(cd,'\',sat(ii).name,'.a');
        root.ExecuteCommand(sprintf('SetState */Satellite/%s FromFile "%s"',sat(ii).name,efile));
        root.ExecuteCommand(sprintf('SetAttitude */Satellite/%s File "%s"',sat(ii).name,afile));
        root.ExecuteCommand(sprintf('VO */Satellite/%s Pass3D OrbitLead None OrbitTrail None',sat(ii).name));
        root.ExecuteCommand(sprintf('VO */Satellite/%s Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/cubesat_6u.dae"',sat(ii).name));
        root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 6U-Cubesat Yaw 0 180',sat(ii).name));
    end
    
    root.ExecuteCommand('VO * ViewFromTo Normal From Satellite/Inspection To Satellite/Inspection');

    
    root.ExecuteCommand('SetAnimation * AnimationMode xRealTime');
    root.ExecuteCommand('Animate * Reset');
end