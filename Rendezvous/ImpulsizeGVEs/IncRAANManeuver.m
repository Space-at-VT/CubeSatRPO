%% Impulsive Inclination and/or RAAN Maneuver
% Author: Dylan Thomas
clearvars; clc; delete('*.asv');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Spacecraft Properties

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Setting up problem constants, etc.

% Parameter Inputs, constants
R_e = 6378.137;                         % [km]          - Radius of Earth
mu = 398600;                            % [km^3/s^2]    - Grav. Parameter
g = 9.81/1000;                          % [km/s^2]      - Gravity
n = 300;                               %               - Num. of Plot points
m_prop_nom = 1.5;                       % [kg]          - Nominal Propellant Mass
inc_target_d = 51.6; %ISS inc           % [deg]         - Target orbit inclination
deg2rad = pi/180;

% Setting up vectorized parameters
init_alt = linspace(300,5000,n);        % [km]          - Initial Orbit Altitude
delta_OM_d = linspace(0,4,n);           % [deg]         - Change in RAAN
delta_i_d = linspace(0,4,n);            % [km]          - Inclination Change
inc_init_d = inc_target_d - delta_i_d;  % [deg]         - Initial orbit inclination
R_i = init_alt + R_e;                   % [km]          - Initial Orbit SMA
period = 2*pi*sqrt(R_i.^3/mu);          % [sec]         - Final Orbital period
h_i = (mu*R_i).^(0.5);                  % [km]          - Orbit angular momentum

% Separate parameters for combined maneuver, plot surface of di vs. dOM on
% a certain number of discrete orbit radiuses 
Discrete_Radius = [6800 7500 10000];

% Convert to radians
inc_target_r = inc_target_d*deg2rad;
inc_init_r = inc_init_d*deg2rad;
delta_i_r = delta_i_d*deg2rad;     
delta_OM_r = delta_OM_d*deg2rad;

% Cases to consider
method = {'Inclination Only Maneuver','RAAN Only Maneuver','Combined Maneuver'};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Building the design space

% Equation for inclination/RAAN change
Dv = @(del_i,del_OM,r,h) (h./r).*(del_i.^2 + (del_OM.^2).*sin(inc_target_r*ones(size(del_i)) - del_i).^2).^(0.5);

% Build matrices for orbital element changes
dIncMat = repmat(delta_i_r,n,1);
dRAANMat = repmat(delta_OM_r',1,n);

% Memory allocation
Dv_Req = zeros(n,n,n);
tic
for ii=1:n
    Dv_Req(:,:,ii) = Dv(dIncMat,dRAANMat,R_i(ii),h_i(ii));
end
toc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Calculate DeltaV & mass percent for each maneuver case

% Memory allocation
Dv_avail = zeros(1,length(method));
Tburn = zeros(1,length(method));
a_thrusti = zeros(1,length(method));
for iter=1:length(method)
    
    % Switch on maneuver case
    maneuver_case  = method{iter};
    switch maneuver_case
        case 'Inclination Only Maneuver'
            display(method(iter))
            
            % Equation for thrust
            a_thrusti(iter) = spacecraft.T_nom/spacecraft.M_total;
            
            % Find total DeltaV & Time of Flight
            Dv_avail(iter) = g*spacecraft.I_sp*log(spacecraft.M_total/(spacecraft.M_total-spacecraft.M_prop));
            Tburn(iter) = (Dv_avail(iter)*1000)/(a_thrusti(iter));
            
            % Memory allocation & reset temporary variables
            Mass_percent = zeros(length(R_i),n);
            Dv_total = Dv_Req(1,:,:);
            
            % Loop calculates mass percent for each radius
            for ii=1:length(R_i)
                
                % Total DeltaV required cannot exceed DeltaV available
                index = Dv_total(1,:,ii)>=Dv_avail(iter);
                Dv_total(1,index,ii) = NaN;
                
                % Determine mass with Rocket Eqn
                M_prop_vec = spacecraft.M_total*(1-exp(-(Dv_total(1,:,ii))/(g*spacecraft.I_sp)));
                Mass_percent(:,ii) = M_prop_vec/m_prop_nom*100;
                
            end
            
            % Plotting stuff
            figure(iter)
            hold on
            grid on
            surf(R_i,delta_i_d,Mass_percent,'EdgeColor','None')
            c = colorbar;
            c.Label.String = 'Percent Propellant Mass Burned';
            title1 = title(method(iter));
            xl = xlabel('Orbit Radius, [km]');
            yl = ylabel('Inclination Change, $\delta i$, [deg]');
            set([title1 xl yl],'interpreter','latex','fontsize',12)
            axis tight
            hold off
          
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 'RAAN Only Maneuver'
            display(method(iter))
            
            % Equation for thrust
            a_thrusti(iter) = spacecraft.T_nom/spacecraft.M_total;
            
            % Find total DeltaV & Time of Flight
            Dv_avail(iter) = g*spacecraft.I_sp*log(spacecraft.M_total/(spacecraft.M_total-spacecraft.M_prop));
            Tburn(iter) = (Dv_avail(iter)*1000)/(a_thrusti(iter));
            
            % Memory allocation & reset temporary variables
            Mass_percent = zeros(length(init_alt),n);
            Dv_total = Dv_Req(:,1,:);
            
            % Loop calculates mass percent for each radius
            for ii=1:length(init_alt)
                
                % Total DeltaV required cannot exceed DeltaV available
                index = Dv_total(:,1,ii)>=Dv_avail(iter);
                Dv_total(index,1,ii) = NaN;
                
                % Determine mass with Rocket Eqn
                M_prop_vec = spacecraft.M_total*(1-exp(-(Dv_total(:,1,ii))/(g*spacecraft.I_sp)));
                Mass_percent(:,ii) = M_prop_vec/m_prop_nom*100;
                
            end
            
            % Plotting stuff
            figure(iter)
            hold on
            grid on
            surf(R_i,delta_OM_d,Mass_percent,'EdgeColor','None')
            c = colorbar;
            c.Label.String = 'Percent Propellant Mass Burned';
            title1 = title(method(iter));
            xl = xlabel('Orbit Radius, [km]');
            yl = ylabel('RAAN Change, $\delta \Omega$, [deg]');
            set([title1 xl yl],'interpreter','latex','fontsize',12)
            axis tight
            hold off
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 'Combined Maneuver'
            display(method(iter))
            
            %%% TODO: Figure this out
            R_i = Discrete_Radius;
            period = 2*pi*sqrt(R_i.^3/mu);
            Dv_Req = zeros(n,n,length(R_i));
            for ii=1:length(R_i)
                Dv_Req(:,:,ii) = Dv(dIncMat,dRAANMat,R_i(ii),h_i(ii));
            end
            
            % Equations for thrust
            a_thrusti(iter) = spacecraft.T_nom/spacecraft.M_total;
            
            % Find total DeltaV & Time of Flight
            Dv_avail(iter) = g*spacecraft.I_sp*log(spacecraft.M_total/(spacecraft.M_total-spacecraft.M_prop));
            Tburn(iter) = (Dv_avail(iter)*1000)/(a_thrusti(iter));
            
            % Memory allocation & reset temporary variables
            Mass_percent = zeros(length(init_alt),n);
            Dv_total = Dv_Req;
            
            % Loop calculates mass percent for each radius
            for ii=1:length(R_i)
                
                % Delta V required cannot exceed Delta V available
                index = Dv_total(:,:,ii)>=Dv_avail(iter);
                for kk=1:n
                    for jj=1:n
                        if Dv_total(kk,jj,ii)>=Dv_avail(ii)
                            Dv_total(kk,jj,ii) = NaN;
                        end
                    end
                end
                
                % Determine mass with Rocket Eqn
                M_prop_vec = spacecraft.M_total*(1-exp(-(Dv_total(:,:,ii))/(g*spacecraft.I_sp)));
                Mass_percent(:,:,ii) = M_prop_vec/m_prop_nom*100;
                
                % Plotting stuff
                figure(iter+ii-1)
                hold on
                grid on
                surf(delta_i_d,delta_OM_d,Mass_percent(:,:,ii),'EdgeColor','None')
                c = colorbar;
                c.Label.String = 'Percent Propellant Mass Burned';
                title1 = title(strcat(method(iter), ', R = ',num2str(R_i(ii))));
                xl = xlabel('Inclination Change, $\delta i$, [deg]');
                yl = ylabel('RAAN Change, $\delta \Omega$, [deg]');
                set([title1 xl yl],'interpreter','latex','fontsize',12)
                axis tight
                hold off
            end         
            
    end

end

