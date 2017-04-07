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
deputy.umax = 0.25;
deputy.Tmax = 0.007;
deputy.vmax = 0.1;
deputy.dryMass = 13;
deputy.fuel = 0.5;
deputy.kp = 0.1;
deputy.kd = 0.1;
deputy.point = 1;
deputy.q3 = 1;
deputy.q4 = 0;

% Proximity holding zone
lbnd = [chief.ubnd(1)+2,chief.lbnd(2),chief.lbnd(3)];
ubnd = [chief.ubnd(1)+10,chief.ubnd(2),chief.ubnd(3)];
center = (ubnd+lbnd)/2;

% Deputy initial state
deputy.x = center(1);
deputy.y = center(2)-1;
deputy.z = center(3);
deputy.vx = -0.05;
deputy.vy = 0.05;
deputy.vz = 0.01;

%% Proximity operations
% Approach
% while separation(deputy.p,dock,1) > tol(1) || separation(deputy.p,dock,2) > tol(2)...
%         || separation(deputy.p,dock,3) > tol(3)
%     deputy.printEphemeris
%     deputy.approach(dock,RSO.lbnd,RSO.ubnd);
% end

% Hold  
thold = 0;
tmax = 100;
hold on
skip = 5;
ii = skip;
while thold < tmax
    deputy.printEphemeris
    deputy.maintain(lbnd,ubnd);
    thold = thold+deputy.scenario.dt;
    ii = ii+1;
    if ii > skip;
        deputy.plotTrajectory(chief.lbnd,chief.ubnd,1)
        ii = 1;
    end  
end