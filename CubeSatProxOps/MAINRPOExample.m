clear,clc
close all

% Time and orbit parameters
scenario = newScenario;
scenario.tmax = 240;
scenario.T = 15;

% Chief bounds
chief = newSatellite;
chief.EOM = 'LERM';
chief.bnd = [2,3,2];

% Deputy parameters
deputy = newSatellite;
deputy.EOM = 'LERM';
deputy.bnd = [0.1,0.3,0.2];
deputy.d = [0.01,0.03,0.01];
deputy.umax = 0.25;
deputy.Tmax = 0.1;
deputy.vmax = 0.25;
deputy.dryMass = 13;
deputy.fuel = 0.5;
deputy.kp = 0.1;
deputy.kd = 0.1;
deputy.makeMovie = 0;

% Deputy initial state
deputy.x = 10;
deputy.y = -50;
deputy.z = 25;
deputy.vy = 0;

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
        deputy = deputy.approach(scenario,center,chief.lbnd,chief.ubnd);
    elseif strcmp(deputy.mode,'maintain')
        deputy = deputy.maintain(scenario,lbnd,ubnd);
    end
    
    clf
    plotTrajectory(deputy,chief.lbnd,chief.ubnd,5);
    
    scenario.t = scenario.t+scenario.dt;
end

plotControls(deputy,scenario)
save('RPOExample')
if deputy.makeMovie
    deputy = deputy.renderFrames;
end
