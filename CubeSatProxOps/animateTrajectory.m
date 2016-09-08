%%
clear,clc
load('SwarmFollow4')

satIter = newSatellite;
lengthSat = zeros(1,length(sat));
for jj = 1:length(sat)
    lengthSat(jj) = length(sat(jj).x);
end    

for ii = 1:min(lengthSat)
    clf
    plotObstacle([chief.x(ii),chief.y(ii),chief.z(ii)]+chief.lbnd,...
        [chief.x(ii),chief.y(ii),chief.z(ii)]+chief.ubnd,'-k');
    for jj = 1:length(sat)
        satIter.x = sat(jj).x(1:ii);
        satIter.y = sat(jj).y(1:ii);
        satIter.z = sat(jj).z(1:ii);
        satIter.ux = sat(jj).ux(1:ii);
        satIter.uy = sat(jj).uy(1:ii);
        satIter.uz = sat(jj).uz(1:ii);
        plotTrajectory(satIter);
    end
    view(-330,30)
    pause(1e-10)
end
%%
load('RPOExample')
satIter = newSatellite;
for ii = 1:length(deputy.x)
    clf
    plotObstacle(chief.lbnd,chief.ubnd,'-k');
    satIter.x = deputy.x(1:ii);
    satIter.y = deputy.y(1:ii);
    satIter.z = deputy.z(1:ii);
    satIter.ux = deputy.ux(1:ii);
    satIter.uy = deputy.uy(1:ii);
    satIter.uz = deputy.uz(1:ii);
    plotTrajectory(satIter);
    pause(1e-10)
end