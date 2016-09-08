clear,clc

load('SwarmFollow4')

chief.name = 'Chief';
createSTKfile(chief,scenario);

for ii = 1:length(sat)
    sat(ii).name = sprintf('Satellite%d',ii);
    createSTKfile(sat(ii),scenario);
end

app = actxserver('STK11.application');
root = app.Personality2;

root.ExecuteCommand('New / Scenario MatlabScenario');

root.ExecuteCommand('New / */Satellite Origin');
root.ExecuteCommand(sprintf('SetState */Satellite/Origin Classical TwoBody UseScenarioInterval 60 ICRF "1 Jan 2000" %f 0 28.5 0 0 0',scenario.a));
root.ExecuteCommand('VO * ObjectStateInWin Show off Object Satellite/Origin WindowId 1');

root.ExecuteCommand(sprintf('New / */Satellite Chief'));
file = strcat(cd,'\Chief.e');
root.ExecuteCommand(sprintf('SetState */Satellite/Chief FromFile "%s"',file));
root.ExecuteCommand('VO */Satellite/Chief Pass3D OrbitLead None OrbitTrail None');

for ii = 1:length(sat)
    root.ExecuteCommand(sprintf('New / */Satellite %s',sat(ii).name));
    file = strcat(cd,'\',sat(ii).name,'.e');
    root.ExecuteCommand(sprintf('SetState */Satellite/%s FromFile "%s"',sat(ii).name,file));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Pass3D OrbitLead None OrbitTrail None',sat(ii).name));
    root.ExecuteCommand(sprintf('VO */Satellite/%s Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/cubesat.mdl"',sat(ii).name));
end

root.ExecuteCommand('SetAnimation * AnimationMode xRealTime');
root.ExecuteCommand('Animate * Reset');