clear,clc
close all

% Time and orbit parameters
scenario = newScenario;
scenario.tmax = 300;
scenario.T = 15;

% Chief bounds
chief = newSatellite;
chief.EOM = 'LERM';
chief.bnd = [2,3,2];

% Deputy parameters
deputy = newSatellite;
deputy.EOM = 'LERM';
deputy.x = 15;
deputy.vz = 0;
deputy.umax = 0.25;
deputy.vmax = 0.25;
deputy.dryMass = 13;
deputy.fuel = 0.5;
deputy.dx = 0;
deputy.dy = 0;
deputy.dz = 0;
deputy.Tmax = 0.025;
deputy.kp = 0;
deputy.kd = 0;

deputy.mode = 'approach';

% Proximity holding zone
lbnd = [chief.ubnd(1)+2,chief.lbnd(2),chief.lbnd(3)];
ubnd = [chief.ubnd(1)+10,chief.ubnd(2),chief.ubnd(3)];
center = (ubnd+lbnd)/2;

iter = 0;
while scenario.t <= scenario.tmax
    iter = iter+1;
    clc
    fprintf('Time: %5.1f\n',scenario.t)
    
    % Mode checks
    if separation(deputy.p,center) < 0.25
        deputy.mode = 'maintain';
        scenario.T = 30;
    end
    if deputy.p(1) > ubnd(1) || deputy.p(2) > ubnd(2) || deputy.p(3) > ubnd(3) ||...
            deputy.p(1) < lbnd(1) || deputy.p(2) < lbnd(2) || deputy.p(3) < lbnd(3)
        deputy.mode = 'approach';
        scenario.T = 15;
    end
    
    % Deputy propagation (MPC)
    if strcmp(deputy.mode,'approach')
        deputy = approach(deputy,scenario,center);
    elseif strcmp(deputy.mode,'maintain')
        deputy = maintain(deputy,scenario,lbnd,ubnd);
    end
    
    % Plot
    clf
    plotObstacle(chief.lbnd,chief.ubnd,'-k');
    plotTrajectory(deputy,0);
    pause(0.0001)

    scenario.t = scenario.t+scenario.dt;
end
plotControls(deputy,scenario)
save('RPOExample')