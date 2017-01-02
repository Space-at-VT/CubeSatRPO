clear,clc
close all

%% Scenario
scenario = newScenario;
scenario.T = 20;
scenario.dt = 1;

%% CubeSat
sat = newSatellite(scenario);
sat.EOM = 'LERM';
sat.bnd = [0.1,0.3,0.2];
sat.x = 0;
sat.y = -10;
sat.z = 0;
sat.vmax = 0.5;
sat.umax = 0.25;
sat.Tmax = 0.003;
sat.dryMass = 6;
sat.fuel = 0.5;
sat.kp = 0.05;
sat.kd = 0.05;
sat.point = 0;
sat.pt = [0,0,0];
sat.safety = 0.5;

%% RSO
% Panels
obj.ubnd = [1,1,2
            6,1,0.1
            -1,1,0.1];
obj.lbnd = [-1,-1,-2
            1,-1,-0.1
            -6,-1,-0.1];
        
obj2.ubnd = [1,1,2
            6,0.1,1
            -1,0.1,1];
obj2.lbnd = [-1,-1,-2
             1,-0.1,-1
            -6,-0.1,-1];

%% Waypoint
waypoint = [0,10,0];
tol = [0.25,0.25,0.25];

%% Model Predictive Control
% Approach
hold on
skip = 5;
ii = 6;
drawSatelliteX(1,4,[5,2],5);
light('Position',[100 0 100],'Style','local');
while separation(sat.p,waypoint,1) > tol(1) || separation(sat.p,waypoint,2) > tol(2)...
        || separation(sat.p,waypoint,3) > tol(3)
    sat.printEphemeris
    sat.approach(waypoint,obj.lbnd,obj.ubnd);

%     if sat.t(end) > 25
%         obj = obj2;
%         drawSatellite(1,3,[5,2],90);
%     end

    ii = ii+1;
    if ii > skip;
        sat.plotTrajectory(obj.lbnd,obj.ubnd,1);
        ii = 1;
    end  
end

% Holding
% thold = 0;
% tmax = 30;
% while thold < tmax
%     sat.maintain(waypoint-tol,waypoint+tol);
%     thold = thold+sat.scenario.dt;
% end

sat.fuelUsed*1e3
sat.dv(end)


%% Post-Process
%sat.renderVideo('AIAA1.avi',obj.lbnd,obj.ubnd,1);
sat.plotControls