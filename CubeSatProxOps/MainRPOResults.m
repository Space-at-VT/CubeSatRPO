function MainRPOResults
close all

V = (1:25);
hold on
grid on
xlabel('Constrained Volume, m^3')
ylabel('Fuel Used, g')

for ii = 1:length(V)
    try
        deputy = RPO(V(ii),0);
        fuel(ii) = deputy.fuel(1)-deputy.fuel(end);
    catch
        fuel(ii) = NaN;
    end
    plot(V(ii),fuel(ii)*1e3,'ob')
    drawnow
    save('npData','fuel')
end

for ii = 1:length(V)
    try
       deputy = RPO(V(ii),1);
       fuelp(ii) = deputy.fuel(1)-deputy.fuel(end);
    catch
       fuelp(ii) = NaN;
    end
    plot(V(ii),fuelp(ii)*1e3,'rx')
    drawnow
    save('pData','fuelp')
end

% figure
% hold on
% p1 = plot(V,fuel*1e3,'ob');
% p2 = plot(V,fuelp*1e3,'xr');
% grid on
% xlabel('Constrained Volume, m^3')
% ylabel('Fuel Used, g')
% legend([p1,p2],{'No Attitude Pointing','Targeted Attitude Pointing'},...
%     'location','best')
end

function deputy = RPO(V,point)
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
%deputy.d = [0.001,0.003,0.002];
deputy.umax = 0.25;
deputy.Tmax = 0.007;
deputy.vmax = 0.1;
deputy.dryMass = 13;
deputy.fuel = 0.5;
deputy.kp = 0.1;
deputy.kd = 0.1;
deputy.point = point;

x0 = 1;
% x = 10;
x = V^(1/3);
% x = V;

% Proximity holding zone
lbnd = [chief.ubnd(1)+x0,-x/2,-x/2];
ubnd = [chief.ubnd(1)+x0+x,x/2,x/2];
% lbnd = [chief.ubnd(1)+x0,chief.lbnd(2),chief.lbnd(3)];
% ubnd = [chief.ubnd(1)+x0+x,chief.ubnd(2),chief.ubnd(3)];

%center = [x*rand+x0,x*rand(1,2)-x/2];
center = (ubnd+lbnd)/2;

% Deputy initial state
deputy.x = center(1);
deputy.y = center(2);
deputy.z = center(3);

% Proximity operations
tmax = scenario.TP;
while true
    deputy.printEphemeris
    deputy.maintain(lbnd,ubnd);    
    if deputy.t(end) > tmax,break,end
end
end


%     if deputy.x(end) < ubnd(1) && deputy.y(end) < ubnd(2) && deputy.z(end) < ubnd(3)...
%             && deputy.x(end) > lbnd(1) && deputy.y(end) > lbnd(2) && deputy.z(end) > lbnd(3)
%         deputy.scenario.dt = 1;
%         deputy.T = 15;
%     else
%         deputy.scenario.dt = 1;
%         deputy.T = 15;
%         deputy.approach(center,chief.lbnd,chief.ubnd);
%     end