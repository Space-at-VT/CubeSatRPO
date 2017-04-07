%% Bare bones version of Energey Management Unit
% Calculates the state of the charge of a battery given power generation
% from STK and power usage.
function simpleEMU(fileName)
% Battery parameters
SOC = 0.5;                  % State of charge (0-1)
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
load(fileName)
eff = 1;
pPanel = pPanel*eff;

%% Initial conditions
SOC40 = SOC;
Emax = 40*3600;             % Battery capacity (J)
Eavail = SOC40*Emax;
Ibatt = 0;
% Iteration
for ii = 1:(length(time)-1)
    % Available power
    pAvail(ii) = pPanel(ii)-pLoad(ii);
    
    if pAvail(ii) < 0
        Zbatt = ZbattDCR; % Discharging
    else
        Zbatt = ZbattCR; % Charging
    end
    
    % Battery voltage
    Vbatt_OC(ii) = nSeries*(k6*SOC40(ii)^6+k5*SOC40(ii)^5+k4*SOC40(ii)^4+...
        k3*SOC40(ii)^3+k2*SOC40(ii)^2+k1*SOC40(ii)+k0);
    Vbatt(ii) = Vbatt_OC(ii)+Ibatt(ii)*Zbatt;
    
    % Available energy
    dt = time(ii+1)-time(ii); 
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
    SOC40(ii+1) = Eavail(ii+1)/Emax;
end

%% Initial conditions
SOC80 = SOC;
Emax = 80*3600;             % Battery capacity (J)
Eavail = SOC80*Emax;
Ibatt = 0;
% Iteration
for ii = 1:(length(time)-1)
    % Available power
    pAvail(ii) = pPanel(ii)-pLoad(ii);
    
    if pAvail(ii) < 0
        Zbatt = ZbattDCR; % Discharging
    else
        Zbatt = ZbattCR; % Charging
    end
    
    % Battery voltage
    Vbatt_OC(ii) = nSeries*(k6*SOC80(ii)^6+k5*SOC80(ii)^5+k4*SOC80(ii)^4+...
        k3*SOC80(ii)^3+k2*SOC80(ii)^2+k1*SOC80(ii)+k0);
    Vbatt(ii) = Vbatt_OC(ii)+Ibatt(ii)*Zbatt;
    
    % Available energy
    dt = time(ii+1)-time(ii); 
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
    SOC80(ii+1) = Eavail(ii+1)/Emax;
end

%% Initial conditions
SOC120 = SOC;
Emax = 120*3600;             % Battery capacity (J)
Eavail = SOC120*Emax;
Ibatt = 0;
% Iteration
for ii = 1:(length(time)-1)
    % Available power
    pAvail(ii) = pPanel(ii)-pLoad(ii);
    
    if pAvail(ii) < 0
        Zbatt = ZbattDCR; % Discharging
    else
        Zbatt = ZbattCR; % Charging
    end
    
    % Battery voltage
    Vbatt_OC(ii) = nSeries*(k6*SOC80(ii)^6+k5*SOC120(ii)^5+k4*SOC120(ii)^4+...
        k3*SOC120(ii)^3+k2*SOC120(ii)^2+k1*SOC120(ii)+k0);
    Vbatt(ii) = Vbatt_OC(ii)+Ibatt(ii)*Zbatt;
    
    % Available energy
    dt = time(ii+1)-time(ii); 
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
    SOC120(ii+1) = Eavail(ii+1)/Emax;
end


%% Plots
TP = 5987.852844;

figure('Position',[100,100,1280,720])
subplot(2,1,1)
hold on
plot(time/TP,pPanel,'LineWidth',1.5)
plot(time/TP,pLoad,'LineWidth',1.5)
plot(time/TP,pPanel-pLoad,'LineWidth',1.5)
hold off
grid on
axis([0 time(end)/TP 0 1],'auto y')
xlabel('Time, Orbits','FontSize',14)
ylabel('Watts','FontSize',14)
legend({'Solar Power Generation','Power Usage','Net Power'},...
    'location','northoutside','Orientation','horizontal','FontSize',12)

subplot(2,1,2)
hold on
plot(time/TP,SOC40*100,'LineWidth',1.5)
% plot(time/TP,SOC80*100,'LineWidth',1.5)
% plot(time/TP,SOC120*100,'LineWidth',1.5)
hold off
grid on
axis([0 time(end)/TP 0 100])
xlabel('Time, Orbits','FontSize',14)
ylabel('State of Charge, %','FontSize',14)
legend({'40 Whr Battery'},'location','northoutside','Orientation','horizontal','FontSize',12)
% legend({'40 Whr Battery','80 Whr Battery','120 Whr Battery'},...
%     'location','northoutside','Orientation','horizontal','FontSize',12)

% figure
% hold on
% plot(time/TP,Vbatt,'LineWidth',1.5)
% plot(time/TP,Ibatt,'LineWidth',1.5)
% hold off
% grid on
% legend({'Voltage, Volts','Current, Amps'},...
%     'location','best')