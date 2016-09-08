clear,clc
close all

scenario = scenarioOrigin;
scenario.tmax = 1000;
scenario.T = 15;
scenario.ecc = 0.9;

sat = satellite;
sat.mode = 'approach';
sat.vmax = 0.5;
sat.EOM = 'LERM';


p1 = [30,-30,10];
p2 = [30,30,10];
p3 = [-30,30,-10];
p4 = [-30,-30,-10];
p = p1;

iter = 0;
while scenario.t < scenario.tmax
    iter = iter+1;
    
    if separation([sat.x(end),sat.y(end),sat.z(end)],p1) < 1
        p = p2;
    elseif separation([sat.x(end),sat.y(end),sat.z(end)],p2) < 1
        p = p3;
    elseif separation([sat.x(end),sat.y(end),sat.z(end)],p3) < 1
        p = p4;
    elseif separation([sat.x(end),sat.y(end),sat.z(end)],p4) < 1
        p = p1;
    end
    
    sat = sat.approach(scenario,p);
    
    displaySat(sat,scenario)
    
    clf
    plotTrajectory(sat(1));
    view(0,90)
    pause(0.001)
    scenario.t = scenario.t+scenario.dt;
end


