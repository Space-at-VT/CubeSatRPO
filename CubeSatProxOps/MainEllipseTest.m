clear,clc
close all

% Time and orbit parameters
scenario = newScenario;
scenario.T = 15;
scenario.a = 7000e3;
scenario.dt = 10;

% RSO bounds
RSO = newSatellite(scenario);
RSO.bnd = [0.1,0.2,0.1];

% Deputy parameters
sat = newSatellite(scenario);
sat.EOM = 'LERM';
sat.bnd = [0.1,0.3,0.2];
sat.umax = 0;
sat.Tmax = 0.007;
sat.dryMass = 11.5;
sat.fuel = 0.5;
sat.kp = 0;
sat.kd = 0;
sat.point = 0;
sat.pt = [0,0,0];

% Deputy initial state
sat.x = 20;
sat.y = 0;
sat.z = 0;
sat.vy = -2*scenario.n*sat.x;
tspan = sat.scenario.TP;

% First relative ellipse
% while sat.t(end) < tspan
%     sat.maintain([-100,-100,-100],[100,100,100]);
%     clf
%     sat.plotTrajectory([],[],5)
% end
sat.propagate(tspan);
RSO.propagate(sat.t(end));
sat.subplotTrajectory;
