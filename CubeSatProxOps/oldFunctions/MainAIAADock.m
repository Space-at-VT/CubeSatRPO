clear,clc
close all

%% Initialization
% Time and orbit parameters
scenario = newScenario;
scenario.T = 15;
scenario.a = 7000e3;

% RSO bounds
RSO = newSatellite(scenario);
RSO.bnd = [0.1,0.2,0.1];

% Deputy parameters
sat = newSatellite(scenario);
sat.EOM = 'LERM';
sat.bnd = [0.1,0.3,0.2];
sat.Tmax = 0.001;
sat.vmax = 0.25;
sat.dryMass = 6;
sat.fuel = 0.5;
sat.kp = 0.05;
sat.kd = 0.05;
sat.point = 1;
sat.pt = [0,0,0];

% Low thrust monoprop (hydrazine)
sat.umax = 0.1;
sat.Isp = 150;

% Deputy initial state
sat.x = 200;
sat.y = -100;
sat.z = 100;
tspan = sat.scenario.TP;

% obstacles
obj.ubnd = [2,1,1
            0.1,1,6
            0.1,1,-1];
obj.lbnd = [-2,-1,-1
            -0.1,-1,1
            -0.1,-1,-6];

%% Rendezvous
% First relative ellipse
x0 = 100;
Xf = [x0,0,0,0,-2*sat.scenario.n*x0,0.01];
sat.phaseManeuverEq(Xf,tspan,15);
sat.propagate(tspan);

% First relative ellipse
x0 = 10;
Xf = [x0,0,0,0,-2*sat.scenario.n*x0,0];
sat.phaseManeuverEq(Xf,tspan,15);
sat.propagate(tspan);

%% Proximity operations
dock = [2.5,0,0];
tol = [0.1,0.1,0.1];

% Approach
% while separation(sat.p,dock,1) > tol(1) || separation(sat.p,dock,2) > tol(2)...
%         || separation(sat.p,dock,3) > tol(3)
%     sat.printEphemeris
%     sat.approach(dock,obj.lbnd,obj.ubnd);
% 
% end

% % Hold  
% thold = 0;
% tmax = 60;
% while thold < tmax
%     sat.printEphemeris
%     sat.maintain(dock-tol,dock+tol);
%     thold = thold+sat.scenario.dt;
% end

RSO.propagate(sat.t(end));

%% Post process
close all
drawSatelliteX(1,4,[5,2],5);
light('Position',[100 0 100],'Style','local');
sat.plotTrajectory(obj.lbnd,obj.ubnd,1);
sat.subplotTrajectory
%sat.renderVideo('AIAA2.avi',obj.lbnd,obj.ubnd,20);