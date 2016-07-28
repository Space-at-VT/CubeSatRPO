%% AOE 5234 Project - Semi-major Axis Change
% Dylan Thomas
clearvars; clc; delete('*.asv');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Spacecraft Properties
f1 = 'Name';
f2 = 'M_total';
f3 = 'M_prop';
f4 = 'I_sp';
f5 = 'T_nom';

v1 = {'Busek Electrospray','Lunar IceCube','Aerojet GR-1','Aerojet CHAMPS-120XW'};
v2 = {14,                   14,             14,            14};      % [kg]
v3 = {1.5,                  1.5,            1.5,           1.5};     % [kg]
v4 = {800,                  3500,           235,           203};     % [s]
v5 = {0.0007,               0.0014,         1,             2.79};  % [N]

spacecraft = struct(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameter Inputs
r_E = 6378;                 % [km]          - Radius of Earth
mu = 398600;                    % [km^3/s^2]    - Grav. Parameter
k_tgt = 1;                      %               - Num. of Target Orbits        
k_int = 1;                      %               - Number of Intercept Orbits
g = 9.81/1000;                  % [km/s^2]      - Gravity
n = 1000;                        %               - Num. of Plot points
m_prop_nom = 1.5;               % [kg]          - Nominal Propellant Mass

% Setting up vectorized parameters
alt = linspace(300,1500,n);     % [km]          - Orbit Altitude
DeltaSMA = linspace(0,1000,n);  % [km]          - Semi-Major Axis Change
R_f = alt+r_E;                  % [km]          - Final Orbit Radius
period = 2*pi*sqrt(R_f.^3/mu);

% Create mesh of varying parameters
[X,Y]=meshgrid(R_f,DeltaSMA);
R_i = X-Y;                      % [km]          - Initial Orbit Radius

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Equation for Delta V of Hohmann Transfer
Dv = @(sma,rf) abs(sqrt((2*mu)./(rf-sma)-(2*mu)./((rf-sma)+rf))- ...
    sqrt(mu./(rf-sma)))+abs(sqrt(mu./rf)-sqrt((2*mu)./rf-(2*mu)./((rf-sma)+rf)));    % [km/s]

% Memory allocation
Dv_Req = zeros(n,n);
for i=1:n
    % Loop creates 3-D surface of Delta V required for various transfers
    Dv_Req(i,:) = Dv(DeltaSMA(i),R_f);

end
% Remove initial orbits with altitude less than 200 km
Dv_Req(R_i <=r_E+200)=NaN;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Loops through each spacecraft & calcs/plots

% Memory allocation
Dv_tot = zeros(1,length(spacecraft));
Tburn = zeros(1,length(spacecraft));
a_thrusti = zeros(1,length(spacecraft));
for ii=1:length(spacecraft)
    % Equations for continuous thrust
    m_dot = spacecraft(ii).T_nom/(g*1000*-spacecraft(ii).I_sp);
    a_thrusti(ii) = spacecraft(ii).T_nom/spacecraft(ii).M_total;
    Dv_avail = 0.5*period*a_thrusti(ii)/1000;           % Eqn 6-46
    
    % Find total Delta V & Time of Flight
    Dv_tot(ii) = g*spacecraft(ii).I_sp*log(spacecraft(ii).M_total/(spacecraft(ii).M_total-spacecraft(ii).M_prop));
    Tburn(ii) = (Dv_tot(ii)*1000)/(a_thrusti(ii));
    
    % Memory allocation & reset temporary variables
    Mass_percent = zeros(length(alt),n);
    Dv_total = Dv_Req;
    % Loop calculates Delta V/Mass for each case
    for jj=1:length(alt)
        
        % Checks burn time of flight is not greater than final orbit period
        if Tburn(ii)>0.5*period(jj)
            spacecraft(ii).M_prop = -m_dot*0.5*period(jj); % Eqn 6-45
        else
            Dv_avail(jj)= Dv_tot(ii);
        end
        
        % Delta V required cannot exceed Delta V available
        index = Dv_total(:,jj)>=Dv_avail(jj);
        Dv_total(index,jj) = NaN;
        
        % Propellant mass used, index out unachieveable transfers
        M_prop_vec = spacecraft(ii).M_total*(1-exp(-(Dv_total(:,jj))/(g*spacecraft(ii).I_sp)));
        Mass_percent(:,jj) = M_prop_vec/m_prop_nom*100;

    end
    

    % Plotting stuff
    figure(ii)
    hold on
    grid on
    mesh(alt,DeltaSMA,Mass_percent)
    c = colorbar;
    c.Label.String = 'Percent Propellant Mass Burned';
    title1 = title([spacecraft(ii).Name ' Thruster']);
    xl = xlabel('Final Orbital Altitude, [km]');
    yl = ylabel('Semi-Major Axis Change, $\Delta a$, [km]');
    set([title1 xl yl],'interpreter','latex','fontsize',12)
    axis tight
    hold off
end

