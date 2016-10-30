clear,clc
close all

load('RPOExample')
satIter = newSatellite;
for ii = 1:300
    clf
    plotObstacle(chief.lbnd,chief.ubnd,'-k');
    satIter.x = deputy.x(1:ii);
    satIter.y = deputy.y(1:ii);
    satIter.z = deputy.z(1:ii);
    satIter.ux = deputy.ux(1:ii);
    satIter.uy = deputy.uy(1:ii);
    satIter.uz = deputy.uz(1:ii);
    plotTrajectory(satIter,0);
    pause(1e-10)
end

%%
load('ShuttleInspection')

satIter = newSatellite;
for ii = 1:length(sat.x)
    clf
    for jj = 1:size(shuttleLbnd,1)
        plotObstacle(shuttleLbnd(jj,:),shuttleUbnd(jj,:),'-k');
    end
    plotShuttle(0,0,0,0,0,0,0.05,1e-3,[1,1,0.5])
    
    satIter.x = sat.x(1:ii);
    satIter.y = sat.y(1:ii);
    satIter.z = sat.z(1:ii);
    satIter.ux = sat.ux(1:ii);
    satIter.uy = sat.uy(1:ii);
    satIter.uz = sat.uz(1:ii);
    satIter.q1 = sat.q1(ii);
    satIter.q2 = sat.q2(ii);
    satIter.q3 = sat.q3(ii);
    satIter.q4 = sat.q4(ii);
    plotTrajectory(satIter);
    
    view(230,20)
    pause(1e-3)
end


% %%
% load('SwarmFollow4')
% 
% satIter = newSatellite;
% lengthSat = zeros(1,length(sat));
% for jj = 1:length(sat)
%     lengthSat(jj) = length(sat(jj).x);
% end    
% 
% for ii = 1:min(lengthSat)
%     clf
%     plotObstacle([chief.x(ii),chief.y(ii),chief.z(ii)]+chief.lbnd,...
%         [chief.x(ii),chief.y(ii),chief.z(ii)]+chief.ubnd,'-k');
%     for jj = 1:length(sat)
%         satIter.x = sat(jj).x(1:ii);
%         satIter.y = sat(jj).y(1:ii);
%         satIter.z = sat(jj).z(1:ii);
%         satIter.ux = sat(jj).ux(1:ii);
%         satIter.uy = sat(jj).uy(1:ii);
%         satIter.uz = sat(jj).uz(1:ii);
%         plotTrajectory(satIter,0);
%     end
%     view(-330,30)
%     pause(1e-10)
% end