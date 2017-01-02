function mainAIAA2
clear,clc
close all

scenarioData = newScenario;
tspan = scenarioData.TP*(0.05:0.05:2);
format shorte

% Sweep
% dv = zeros(1,length(tspan));
% for ii = 1:length(tspan)
%     dv(ii) = maneuver(tspan(ii));
% end
% 
% figure(1)
% plot(tspan/scenarioData.TP,dv,'k','linewidth',1.5)
% grid on
% xlabel('Time-of-flight, TP')
% ylabel('\Deltav, m/s')
% 
% save('AIAAz')

%2*scenarioData.n*10*2
% maneuver(1.75*scenarioData.TP)
end

function dv = maneuver(tspan)
scenario = newScenario;
scenario.T = 20;
scenario.dt = 1;

% CubeSat
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

Xf = [0,10,0,0,0,0];
sat.phaseManeuverEq(Xf,tspan,15);

%fuelused = (sat.fuel(1)-sat.fuel(end))*1e3;
dv = sat.dv(end);

%sat.plotTrajectory([0,0,0],[0,0,0],1);

end