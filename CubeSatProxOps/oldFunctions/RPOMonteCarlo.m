function RPOMonteCarlo
close all
clc


x0 = 200;
y0 = -100;
z0 = 100;
% unc = 0.25;
% xrange = [x0*(1-unc),x0*(1+unc)];
% yrange = [y0*(1-unc),y0*(1+unc)];
% zrange = [z0*(1-unc),z0*(1+unc)];
unc = 50;
xrange = [x0-unc,x0+unc];
yrange = [y0-unc,y0+unc];
zrange = [z0-unc,z0+unc];

hold on
for iter = 1:100
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
    
    deputy.x = (xrange(2)-xrange(1))*rand+xrange(1);
    deputy.y = (yrange(2)-yrange(1))*rand+yrange(1);
    deputy.z = (zrange(2)-zrange(1))*rand+zrange(1);
    tspan = deputy.scenario.TP;
    
    Xf = [40,0,0,0,-2*deputy.scenario.n*40,0.01];
    deputy.phaseManeuverEq(Xf,tspan,30);

    deputy.plotTrajectory(chief.lbnd,chief.ubnd,0);
    
    fuel(iter) = deputy.fuel(1)-deputy.fuel(end);
    dv(iter) = deputy.dv(end);
end
deputy.propagate(tspan);

% Second relative ellipse
Xf = [5,0,0,0,-2*deputy.scenario.n*5,0];
deputy.phaseManeuverEq(Xf,tspan,10);
deputy.propagate(tspan);
deputy.plotTrajectory(chief.lbnd,chief.ubnd,0);
    
max(fuel)
min(fuel)
mean(fuel)


