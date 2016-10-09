clear,clc
close all

% Time and orbit parameters
scenario = newScenario;
scenario.tmax = 60;
scenario.T = 15;

% Deputy parameters
deputy = newSatellite;
deputy.EOM = 'LERM';
deputy.bnd = [0.1,0.3,0.2];
deputy.d = [0,0,0];
deputy.umax = 0.25;
deputy.Tmax = 0.004;
deputy.vmax = 30;
deputy.dryMass = 13;
deputy.fuel = 0.5;
deputy.kp = 0;
deputy.kd = 0;
% deputy.q1 = 0.707106781186547;
% deputy.q4 =  0.707106781186547;
deputy.wb1 = 0.1;
% deputy.wb2 = 0.5;
% deputy.wb3 = -0.5;

p = [0,1000,0];

iter = 0;
while scenario.t <= scenario.tmax
    iter = iter+1;
    clc
    fprintf('Time: %5.1f\n',scenario.t)

    deputy = deputy.approach(scenario,p);
    
    clf
    plotTrajectory(deputy,[],[],1);
    
    scenario.t = scenario.t+scenario.dt;
end

plotControls(deputy,scenario)

