clear,clc
close all

scenario = newScenario;
scenario.tmax = 300;
scenario.T = 15;

for ii = 1:6
    sat(ii) = newSatellite;
    sat(ii).y = -25;
    sat(ii).vmax = 0.75;
end

chief = newSatellite;
chief.bnd = [2,3,2];
chief.vmax = 0.25;

d = 5;
p(1,:) = chief.p+[chief.ubnd(1)+d,0,0];
p(2,:) = chief.p+[chief.lbnd(1)-d,0,0];
p(3,:) = chief.p+[0,chief.ubnd(2)+d,0];
p(4,:) = chief.p+[0,chief.lbnd(2)-d,0];
p(5,:) = chief.p+[0,0,chief.ubnd(3)+d];
p(6,:) = chief.p+[0,0,chief.lbnd(3)-d];

iter = 0;
while scenario.t <= scenario.tmax
    iter = iter+1;
    clc
    fprintf('Time: %5.1f\n',scenario.t)
    
    if scenario.t > 100 && scenario.t < 200
        chief = chief.approach(scenario,[50,0,0]);
    elseif scenario.t > 200
        chief = chief.approach(scenario,[50,0,50]);
    else
        chief = chief.approach(scenario,[0,0,0]);
    end
    p(1,:) = chief.p+[chief.ubnd(1)+d,0,0];
    p(2,:) = chief.p+[chief.lbnd(1)-d,0,0];
    p(3,:) = chief.p+[0,chief.ubnd(2)+d,0];
    p(4,:) = chief.p+[0,chief.lbnd(2)-d,0];
    p(5,:) = chief.p+[0,0,chief.ubnd(3)+d];
    p(6,:) = chief.p+[0,0,chief.lbnd(3)-d];
    
    clf
    plotObstacle(chief.p+chief.lbnd,chief.p+chief.ubnd,'-k');
    for ii = 1:length(sat)
        sat(ii) = sat(ii).approach(scenario,p(ii,:),chief.p+chief.lbnd,chief.p+chief.ubnd);
        plotTrajectory(sat(ii));
    end
    pause(0.0001)

    scenario.t = scenario.t+scenario.dt;
end

save('SwarmFollow4')