clear,clc
close all

scenario = hcwScenario;
scenario.tmax = 420;
scenario.T = 15;

for ii = 1:6
    sat(ii) = satellite;
    sat(ii).y = -25;
    sat(ii).vmax = 0.75;
end

chief = satellite;
chief.lbnd = [-1,-1,-1.5];
chief.ubnd = [1,1,1.5];

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
    
    clf
    plotObstacle(chief.lbnd,chief.ubnd,'-k');
    for ii = 1:length(sat)
        sat(ii) = sat(ii).approach(scenario,p(ii,:),chief.lbnd,chief.ubnd);
        plotTrajectory(sat(ii));
    end
    pause(0.0001)
    
    
    if scenario.t == 120
        p = p([3,4,5,6,1,2],:);
        sat(3).vmax = 0.5;
    end
    if scenario.t == 200
        p = 3*p;
    end
    if scenario.t == 280
        for ii = 1:6
            p(ii,:) = [0,25,0];
        end
    end
    
    scenario.t = scenario.t+scenario.dt;
end

% save('SwarmReposition2')