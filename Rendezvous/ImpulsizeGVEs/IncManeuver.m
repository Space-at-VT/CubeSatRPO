%% Impulsive Inclination Change
% Dylan Thomas
clearvars; clc; delete('*.asv');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Spacecraft Properties
%%% Needed?
f1 = 'Name';
f2 = 'M_total';
f3 = 'M_prop';
f4 = 'I_sp';
f5 = 'T_nom';

v1 = {'Aerojet CHAMPS-120XW'};
v2 = {14};      % [kg]
v3 = {1.5};     % [kg]
v4 = {203};     % [s]
v5 = {4*2.79};  % [N]

spacecraft = struct(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5);

% Cases
method = {'Inc Only' , 'RAAN Only', 'Combined Inc & RAAN'};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameter Inputs, constants
r_E = 6378.137;                             % [km]          - Radius of Earth
mu = 398600;                                % [km^3/s^2]    - Grav. Parameter
g = 9.81/1000;                              % [km/s^2]      - Gravity
n = 100;                                   %               - Num. of Plot points
m_prop_nom = 1.5;                           % [kg]          - Nominal Propellant Mass
del_OM_d = linspace(0,20,n);                                 % [deg]         - Change in RAAN
inc_target_d = 51.6; %ISS inc               % [deg]         - Target orbit inclination
deg2rad = pi/180;

% Setting up vectorized parameters
% R_f = 10000;                              % [km]          - Final Orbit SMA
init_alt = linspace(300,5000,n);            % [km]          - Initial Orbit Altitude
del_inc_d = linspace(0,5,n);                % [km]          - Inclination Change
R_i = init_alt + r_E;                       % [km]          - Initial Orbit SMA
period = 2*pi*sqrt(R_i.^3/mu);              % [sec]         - Final Orbital period
inc_init_d = inc_target_d - del_inc_d;      % [deg]         - Initial orbit inclination
h_i = (mu*R_i).^(0.5);                        % [km]          - Orbit angular momentum

% Convert to radians
inc_target_r = inc_target_d*deg2rad;
inc_init_r = inc_init_d*deg2rad;
del_inc_r = del_inc_d*deg2rad;     
del_OM_r = del_OM_d*deg2rad;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Equation for inclination change

Dv = @(del_i,del_OM,r,h) (h/r)*(del_i.^2 + (del_OM.^2)*sin(inc_target_r - del_i).^2).^(0.5);
% Memory allocation
Dv_Req = zeros(n,n,n);
for kk=1:n
    for jj=1:n
        for ii=1:n
            % Loop creates 3-D surface of Delta V required for various transfers
            Dv_Req(kk,jj,ii) = Dv(del_inc_r(jj),del_OM_r(kk),R_i(ii),h_i(ii));
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Loops through each spacecraft & calcs/plots

% Memory allocation
Dv_tot = zeros(1,length(method));
Tburn = zeros(1,length(method));
a_thrusti = zeros(1,length(method));
for iter=1:2
    maneuver_case  = method{iter};
    switch maneuver_case
        case 'Inc Only'
            % Equations for Continuous thrust
            m_dot = spacecraft.T_nom/(g*1000*-spacecraft.I_sp);
            a_thrusti(iter) = spacecraft.T_nom/spacecraft.M_total;
            Dv_avail = 0.5*period*a_thrusti(iter)/1000;           % Eqn 6-46
            
            % Find total Delta V & Time of Flight
            Dv_tot(iter) = g*spacecraft.I_sp*log(spacecraft.M_total/(spacecraft.M_total-spacecraft.M_prop));
            Tburn(iter) = (Dv_tot(iter)*1000)/(a_thrusti(iter));
            
            % Memory allocation & reset temporary variables
            Mass_percent = zeros(length(init_alt),n);
            Dv_total = Dv_Req(1,:,:);
            % Loop calculates Inclination/Mass for each case
            for ii=1:length(init_alt)
                
                % Checks burn time of flight is not greater than final orbit period
                if Tburn(iter)>0.5*period(ii)
                    spacecraft.M_prop = -m_dot*0.5*period(ii); % Eqn 6-45
                else
                    Dv_avail(ii)= Dv_tot(iter);
                end
                
                % Delta V required cannot exceed Delta V available
                index = Dv_total(:,ii)>=Dv_avail(ii);
                Dv_total(index,ii) = NaN;
                
                M_prop_vec = spacecraft.M_total*(1-exp(-(Dv_total(:,ii))/(g*spacecraft.I_sp)));
                Mass_percent(:,ii) = M_prop_vec/m_prop_nom*100;
                
            end
            
        case 'RAAN Only'
            % Equations for Continuous thrust
            m_dot = spacecraft.T_nom/(g*1000*-spacecraft.I_sp);
            a_thrusti(iter) = spacecraft.T_nom/spacecraft.M_total;
            Dv_avail = 0.5*period*a_thrusti(iter)/1000;           % Eqn 6-46
            
            % Find total Delta V & Time of Flight
            Dv_tot(iter) = g*spacecraft.I_sp*log(spacecraft.M_total/(spacecraft.M_total-spacecraft.M_prop));
            Tburn(iter) = (Dv_tot(iter)*1000)/(a_thrusti(iter));
            
            % Memory allocation & reset temporary variables
            Mass_percent = zeros(length(init_alt),n);
            Dv_total = Dv_Req;
            % Loop calculates Inclination/Mass for each case
            for ii=1:length(init_alt)
                
                % Checks burn time of flight is not greater than final orbit period
                if Tburn(iter)>0.5*period(ii)
                    spacecraft.M_prop = -m_dot*0.5*period(ii); % Eqn 6-45
                else
                    Dv_avail(ii)= Dv_tot(iter);
                end
                
                % Delta V required cannot exceed Delta V available
                index = Dv_total(1,:,ii)>=Dv_avail(ii);
                Dv_total(1,index,ii) = NaN;
                
                M_prop_vec = spacecraft.M_total*(1-exp(-(Dv_total(1,:,ii))/(g*spacecraft.I_sp)));
                Mass_percent(:,ii) = M_prop_vec/m_prop_nom*100;
                
            end
    end
    
    %%%% TODO: 

    % Plotting stuff
    figure(iter)
    hold on
    grid on
    mesh(init_alt,del_inc_d,Mass_percent)
    c = colorbar;
    c.Label.String = 'Percent Propellant Mass Burned';
    title1 = title(method(iter));
    xl = xlabel('Orbital Altitude, [km]');
    yl = ylabel('Inclination Change, $\Delta i$, [deg]');
    set([title1 xl yl],'interpreter','latex','fontsize',12)
    axis tight
    hold off
end

