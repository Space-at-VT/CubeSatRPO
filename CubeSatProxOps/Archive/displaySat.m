function displaySat(sat,scenario)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
clc
fprintf('Satellite %d\n',1)
fprintf('Scenario time: %6.1f s\n',scenario.t)
fprintf('--------------------------------\n')
fprintf('Fuel remaining: %7.5f kg\n',sat.fuel(end))
fprintf('x: %6.3f m  vx: %6.3f m/s\n',sat.x(end),sat.vx(end))
fprintf('y: %6.3f m  vy: %6.3f m/s\n',sat.y(end),sat.vy(end))
fprintf('z: %6.3f m  vz: %6.3f m/s\n\n',sat.z(end),sat.vz(end))

end

