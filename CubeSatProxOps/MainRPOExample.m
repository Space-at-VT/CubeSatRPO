clear,clc
close all

% Time and orbit parameters
scenario = newScenario;
scenario.T = 15;
scenario.a = 7000e3;
tmax = scenario.TP;

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
deputy.x = 40;
deputy.y = 40;
deputy.z = 0;

% Proximity holding zone
lbnd = [chief.ubnd(1)+2,chief.lbnd(2),chief.lbnd(3)];
ubnd = [chief.ubnd(1)+10,chief.ubnd(2),chief.ubnd(3)];
center = (ubnd+lbnd)/2;

while deputy.t(end) < tmax
    clc
    fprintf('Time: %5.1f\n',deputy.t(end))
    
    % Mode checks
    if separation(deputy.p,center) < 0.25
        deputy.mode = 'maintain';
        scenario.T = 40;
    end
    if deputy.p(1) > ubnd(1) || deputy.p(2) > ubnd(2) || deputy.p(3) > ubnd(3) ||...
            deputy.p(1) < lbnd(1) || deputy.p(2) < lbnd(2) || deputy.p(3) < lbnd(3)
        deputy.mode = 'approach';
        scenario.T = 15;
    end
    
    % Deputy propagation (MPC)
    if strcmp(deputy.mode,'approach')
        deputy.approach(center,chief.lbnd,chief.ubnd);
    elseif strcmp(deputy.mode,'maintain')
        deputy.maintain(lbnd,ubnd);
    end
    
    clf
    deputy.plotTrajectory(chief.lbnd,chief.ubnd,3);

end

deputy.plotControls