clear,clc
close all

% Time and orbit parameters
scenario = newScenario;
scenario.T = 15;
scenario.a = 7000e3;

% Chief bounds
chief = newSatellite(scenario);
chief.bnd = [2,3,2];

% Deputy parameters
deputy = newSatellite(scenario);
deputy.EOM = 'LERM';
deputy.bnd = [0.1,0.3,0.2];
deputy.d = [0.001,0.003,0.002];
deputy.umax = 0.05;
deputy.Tmax = 0.007;
deputy.vmax = 0.1;
deputy.dryMass = 13;
deputy.fuel = 0.5;
deputy.kp = 0.1;
deputy.kd = 0.1;
deputy.point = 1;

% Deputy initial state
deputy.x = 20;
deputy.y = 5;
deputy.z = 0;

% Proximity holding zone
lbnd = [chief.ubnd(1)+2,chief.lbnd(2),chief.lbnd(3)];
ubnd = [chief.ubnd(1)+10,chief.ubnd(2),chief.ubnd(3)];
center = (ubnd+lbnd)/2;

%% Proximity operations
thold = 0;
tmax = scenario.TP;
while true
    deputy.printEphemeris

    if deputy.x(end) < ubnd(1) && deputy.y(end) < ubnd(2) && deputy.z(end) < ubnd(3)...
            && deputy.x(end) > lbnd(1) && deputy.y(end) > lbnd(2) && deputy.z(end) > lbnd(3)
        deputy.scenario.dt = 1;
        deputy.maintain(lbnd,ubnd);
        deputy.T = 20;
        thold = thold+deputy.scenario.dt;
    else
        deputy.scenario.dt = 1;
        deputy.T = 15;
        deputy.approach(center,chief.lbnd,chief.ubnd);
        thold = 0;
    end
    
    if thold > tmax,break,end
end

deputy.plotControls
deputy.subplotTrajectory;