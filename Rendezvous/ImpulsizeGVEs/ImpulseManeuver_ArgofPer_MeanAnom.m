%% Impulsive Arg. of Periapsis and/or Mean Anomaly Maneuver
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
num_pts = 300;                                %               - Num. of Plot points
m_prop_nom = 1.5;                       % [kg]          - Nominal Propellant Mass
M_target_d = 70;                        % [deg]         - Target orbit inclination
e = 0;                                  %               - Eccentricity
deg2rad = pi/180;

% Setting up vectorized parameters
init_alt = linspace(300,5000,num_pts);        % [km]          - Initial Orbit Altitude
delta_om_d = 0;                         %               - Change in Arg. of Periapsis
delta_M_d = linspace(0,30,num_pts);           % [km]          - Mean anomaly Change
M_init_d = M_target_d - delta_M_d;      % [deg]         - Initial orbit mean anomaly
R_i = init_alt + R_e;                   % [km]          - Initial Orbit SMA
period = 2*pi*sqrt(R_i.^3/mu);          % [sec]         - Final Orbital period
h_i = (mu*R_i).^(0.5);                  % [km]          - Orbit angular momentum

% Separate parameters for combined maneuver, plot surface of di vs. dOM on
% a certain number of discrete orbit radiuses 
Discrete_Radius = [6800 7500 10000];

% Convert to radians
M_target_r = M_target_d*deg2rad;
M_init_r = M_init_d*deg2rad;
delta_M_r = delta_M_d*deg2rad;     
delta_om_r = delta_om_d*deg2rad;

% Cases to consider
method = {'Arg. of Perigee Only Maneuver','Mean Anomaly Only Maneuver','Combined Maneuver'};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Building the design space

% Equation for inclination/RAAN change
Dvp = @(del_om,del_OM,del_M,a,e) -((mu./a).^0.5.*a)/4.*(((1+e)^2/...
    sqrt(1-e^2)).*(del_om + del_OM*cos(inc)) + del_M);
Dva = @(del_om,del_OM,del_M,a,e) ((mu./a).^0.5.*a)/4.*(((1-e)^2/...
    sqrt(1-e^2)).*(del_om + del_OM*cos(inc)) + del_M);

% Build matrices for orbital element changes
dIncMat = repmat(delta_i_r,num_pts,1);
dRAANMat = repmat(delta_OM_r',1,num_pts);

% Memory allocation
Dv_Req = zeros(num_pts,num_pts,num_pts);
tic
for ii=1:num_pts
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
            Mass_percent = zeros(length(R_i),num_pts);
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
            yl = ylabel('Inclination Change $\delta i$, [deg]');
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
            Mass_percent = zeros(length(init_alt),num_pts);
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
            yl = ylabel('RAAN Change $\delta \Omega$, [deg]');
            set([title1 xl yl],'interpreter','latex','fontsize',12)
            axis tight
            hold off
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 'Combined Maneuver'
            display(method(iter))
            
            %%% TODO: Figure this out
            R_i = Discrete_Radius;
            h_i = (mu*R_i).^(0.5); 
            period = 2*pi*sqrt(R_i.^3/mu);
            Dv_Req = zeros(num_pts,num_pts,length(R_i));
            for ii=1:length(R_i)
                Dv_Req(:,:,ii) = Dv(dIncMat,dRAANMat,R_i(ii),h_i(ii));
            end
            
            % Equations for thrust
            a_thrusti(iter) = spacecraft.T_nom/spacecraft.M_total;
            
            % Find total DeltaV & Time of Flight
            Dv_avail(iter) = g*spacecraft.I_sp*log(spacecraft.M_total/(spacecraft.M_total-spacecraft.M_prop));
            Tburn(iter) = (Dv_avail(iter)*1000)/(a_thrusti(iter));
            
            % Memory allocation & reset temporary variables
            Mass_percent = zeros(length(init_alt),num_pts);
            Dv_total = Dv_Req;
            
            % Loop calculates mass percent for each radius
            for ii=1:length(R_i)
                
                % Delta V required cannot exceed Delta V available
                index = Dv_total(:,:,ii)>=Dv_avail(iter);
                for kk=1:num_pts
                    for jj=1:num_pts
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
                title1 = title(strcat(method(iter), ', $R=',num2str(R_i(ii)),'$'));
                xl = xlabel('Inclination Change $\delta i$, [deg]');
                yl = ylabel('RAAN Change $\delta \Omega$, [deg]');
                set([title1 xl yl],'interpreter','latex','fontsize',12)
                axis tight
                hold off
            end         
            
    end

end
