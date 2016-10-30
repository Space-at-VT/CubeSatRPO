clear,clc
close all

%% Read scenario
scenario = newScenario;
deputy = newSatellite;

fileID = fopen('scenario.txt','r');
ii = 1;
while ~feof(fileID)
    line = fgetl(fileID);
    line = strsplit(line,' ');
    property{ii} = line{1};
    val(ii) = str2double(line{2});
      
    switch property{ii}
        case 'umax'
            deputy.umax = val(ii);
        case 'Tmax'
            deputy.Tmax = val(ii);
        case 'dryMass'
            deputy.dryMass = val(ii);
        case 'fuel'
            deputy.fuel = val(ii); 
        case 'a'
            scenario.a = val(ii);
        case 'ecc'
            scenario.ecc = val(ii);
        case 'inc'
            scenario.Om = val(ii);
        case 'x'
            deputy.x = val(ii);
        case 'y'
            deputy.y = val(ii);
        case 'z'
            deputy.z = val(ii);
        case 'vx'
            deputy.vx = val(ii);
        case 'vy'
            deputy.vy = val(ii);
        case 'vz'
            deputy.vz = val(ii);
    end
    ii = ii+1;  
end
fclose(fileID);

%% Read command list
fileID = fopen('CommandList.txt','r');
ii = 1;
while ~feof(fileID)
    line = fgetl(fileID);
    line = strsplit(line,' ');
    cmd{ii} = line{1};
    t0(ii) = str2double(line{2});
    
    switch cmd{ii}
        case 'R'
            tspan(ii) = str2double(line{3});
            for jj = 1:6
                Xf(jj,ii) = str2double(line{jj+3});
            end
        case 'P'
            tspan(ii) = 0;
            for jj = 1:3
                Xf(jj,ii) = str2double(line{jj+2});
            end
            Xf(4:6,ii) = zeros(1,3);
        case 'Prop'
            tspan(ii) = str2double(line{3});
            Xf(:,ii) = zeros(1,6);
    end
    ii = ii+1;  
end
fclose(fileID);

%% Propagate
chief = newSatellite;
chief.EOM = 'LERM';
chief.bnd = [0.1,0.2,0.1];

for ii = 1:length(t0)
    switch cmd{ii}
        case 'R'
            deputy = deputy.phaseManeuver(scenario,Xf(:,ii)',tspan(ii),10);
        case 'P'
            
        case 'Prop'
            deputy = deputy.propagate(scenario,tspan(ii));
    end    
end

plotTrajectory(deputy,chief.lbnd,chief.ubnd,3); 