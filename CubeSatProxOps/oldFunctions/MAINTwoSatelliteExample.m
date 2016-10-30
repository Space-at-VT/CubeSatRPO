clear,clc
close all

scenario = hcwScenario;
scenario.tmax = 1000;
scenario.T = 15;

sat(1) = satellite;
sat(1).name = 'Pursuer';
sat(1).mode = 'approach';
sat(1).y = -25;
sat(1).vmax = 0.5;
sat(1).umax = 0.5;

sat(2) = satellite;
sat(2).name = 'Evader';
sat(2).mode = 'approach';
sat(2).color = 'b';
sat(2).vmax = 0.75;

p1 = [30,-30,10];
p2 = [30,30,10];
p3 = [-30,30,-10];
p4 = [-30,-30,-10];
p = p1;

iter = 0;
while scenario.t <= scenario.tmax
    iter = iter+1;
       
    sat(1) = sat(1).approach(scenario,sat(2).p);
    
    if separation(sat(2).p,p1) < 1
        p = p2;
    elseif separation(sat(2).p,p2) < 1
        p = p3;
    elseif separation(sat(2).p,p3) < 1
        p = p4;
    elseif separation(sat(2).p,p4) < 1
        p = p1;
    end
    
    sat(2) = sat(2).approach(scenario,p);
    
    clf
    for ii = 1:length(sat)
        plotTrajectory(sat(ii));
    end
    view(0,90)
    pause(0.001)
    
    clc
    fprintf('Scenario time: %6.1f s\n',scenario.t)
    scenario.t = scenario.t+scenario.dt;
end

