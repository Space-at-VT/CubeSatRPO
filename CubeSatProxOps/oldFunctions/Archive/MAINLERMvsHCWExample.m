clear,clc
close all

scenario = scenarioOrigin;
scenario.tmax = 100;
scenario.T = 15;
scenario.ecc = 0;

sat(1) = satellite;
sat(1).EOM = 'HCW';
sat(1).mode = 'approach';
sat(1).vx = 0.5;
sat(1).vz = 0.5;
% sat(1).fuel = 0;

sat(2) = satellite;
sat(2).color = 'g';
sat(2).EOM = 'LERM';
sat(2).mode = 'approach';
sat(2).vx = 0.5;
sat(2).vz = 0.5;
% sat(2).fuel = 0;

p = [0,-10,0];

iter = 0;
while scenario.t <= scenario.tmax
    iter = iter+1;
    clc
    fprintf('Time: %5.1f\n',scenario.t)
    
    sat(1) = sat(1).approach(scenario,p);    
    sat(2) = sat(2).approach(scenario,p);
    
    clf
    for ii = 1:length(sat)
        plotTrajectory(sat(ii));
    end
    pause(0.001)
    
    scenario.t = scenario.t+scenario.dt;
end


