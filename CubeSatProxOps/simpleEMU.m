%% Bare bones version of Energey Management Unit
% Calculates the state of the charge of a battery given power generation
% from STK and power usage.
function simpleEMU()
% Battery parameters
SOC = 0.5;                  % State of charge (0-1)
Emax = 40*3600;             % Battery capacity (J)
nSeries = 2;                % Number of modules
nPacks = 1;                 % Number of battery packs
ZbattCR = 0.1;              % BOL Battery Impedance (Ohm)
ZbattDCR = 0.1;             % BOL Battery Impedance (Ohm)
maxCR = 2.6*2*nPacks;       % Max Battery Charge Rate (A)
maxDCR = -2*5.2*nPacks;     % Max Battery Discharge Rate (A)

% Constants
k6 = -30.155;
k5 = 97.5;
k4 = -123.27;
k3 = 77.946;
k2 = -26.082;
k1 = 4.8735;
k0 = 3.1935;

% Solar panel data
% load('PowerData2.mat')
% load('DetumblePower')
load('FullSolar')

% Initial conditions
Eavail = SOC*Emax;
Ibatt = 0;

% Time step
dt = time(2)-time(1);

%% Iteration
for ii = 1:length(time)
    
    if time(ii) < 60
        pLoad(ii) = pLoad(ii)+10; 
    end
    
    % Available power
    pAvail(ii) = pPanel(ii)-pLoad(ii);
    
    if pAvail(ii) < 0
        Zbatt = ZbattDCR; % Discharging
    else
        Zbatt = ZbattCR; % Charging
    end
    
    % Battery voltage
    Vbatt_OC(ii) = nSeries*(k6*SOC(ii)^6+k5*SOC(ii)^5+k4*SOC(ii)^4+...
        k3*SOC(ii)^3+k2*SOC(ii)^2+k1*SOC(ii)+k0);
    Vbatt(ii) = Vbatt_OC(ii)+Ibatt(ii)*Zbatt;
    
     % Available energy
    Eavail(ii+1) = Eavail(ii)+Ibatt(ii)*Vbatt(ii)*dt;
    if Eavail(ii+1) >= Emax
        Eavail(ii+1) = Emax;
    elseif Eavail(ii+1) < 0
        Eavail(ii+1) = 0;
    end
    
    % Available current
    Iavail = pAvail(ii)/Vbatt(ii);
    if Iavail >= maxCR
        Ibatt(ii+1) = maxCR;
    elseif Iavail <= maxDCR
        Ibatt(ii+1) = maxDCR;
    else
        Ibatt(ii+1) = Iavail;
    end
     
    % State of charge
    SOC(ii+1) = Eavail(ii+1)/Emax;
end
Ibatt(end) = [];
Eavail(end) = [];
SOC(end) = [];


%% Plots
TP = 5.8285e+03;

figure
hold on
plot(time/TP,pPanel,'LineWidth',1.5)
plot(time/TP,pLoad,'LineWidth',1.5)
plot(time/TP,SOC*100,'LineWidth',1.5)
hold off
legend({'Solar Power Generation, Watts','Power Usage, Watts','State of Charge, %'},...
    'location','best')
grid on
axis([0 time(end)/TP 0 100])
xlabel('Time, Orbits','FontSize',14)
title('80 Wh','FontSize',14)

figure
hold on
plot(time/TP,Vbatt,'LineWidth',1.5)
plot(time/TP,Ibatt,'LineWidth',1.5)
hold off
grid on
legend({'Voltage, Volts','Current, Amps'},...
    'location','best')