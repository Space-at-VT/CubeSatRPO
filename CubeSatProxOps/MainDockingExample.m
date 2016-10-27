clear,clc
close all

% Time and orbit parameters
scenario = newScenario;
scenario.T = 15;
scenario.a = 7000e3;

% Chief bounds
chief = newSatellite;
chief.EOM = 'LERM';
chief.bnd = [0.1,0.2,0.1];

% Deputy parameters
deputy = newSatellite;
deputy.EOM = 'LERM';
deputy.bnd = [0.1,0.3,0.2];
% deputy.d = [0.001,0.003,0.002];
deputy.umax = 0.1;
deputy.Tmax = 0.007;
deputy.vmax = 0.1;
deputy.dryMass = 11.5;
deputy.fuel = 0.5;

% Deputy initial state
deputy.x = 0;
deputy.y = -300;
deputy.z = 0;
deputy.vy = 0;

tspan = scenario.TP;

x0 = 100;
Xf = [x0,-300,0,0,-2*scenario.n*x0-0.005,0.25];
deputy.phaseManeuver(scenario,Xf,tspan,10);
deputy.propagate(scenario,4*tspan);

Xf = [-10,0,0,0,0,0];
deputy.phaseManeuver(scenario,Xf,tspan,10);
plotTrajectory(deputy,chief.lbnd,chief.ubnd,3);

% dock = [0,0,1];
% while true
%     clc
%     fprintf('Time: %5.1f\n',deputy.t(end))
%     
%     deputy.approach(scenario,dock,chief.lbnd,chief.ubnd);
%     
%     % Deputy propagation (MPC)
%     if separation(deputy.p,dock) < 0.1,break,end
%             
%     clf
%     deputy.plotTrajectory(chief.lbnd,chief.ubnd,3);
% end

deputy.plotControls
