%%% Impulsive Arg. of Periapsis and/or Mean Anomaly Maneuver
% Author: Dylan Thomas

% Equations for Delta V of maneuvers are derived from Gauss' Variational
% Equations as shown in Schaub and Alfriend's "Impulsive Feedback to
% Establish Specific Mean Orbital Elements of Spacecraft Formations", 2001
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
%%% Setting up problem 

% Parameter Inputs, constants across all codes
R_eq = 6378.137;                        % [km]          - Equatorial Radius of Earth
mu = 398600;                            % [km^3/s^2]    - Gravitational Parameter of Earth
g = 9.81/1000;                          % [km/s^2]      - Acceleration due to Gravity
nominalPropMass = 1.5;                  % [kg]          - Nominal Propellant Mass
num_pts = 300;                          %               - Num. of points
deg2rad = pi/180;                       % [rad/deg]     - Conversion from radians to degrees

% Orbital properties
ecc = 0;                                %               - Orbit Eccentricity (Constant)
init_alt = linspace(300,5000,num_pts);  % [km]          - Initial Orbit Altitude
R_p = init_alt + R_eq;                  % [km]          - Initial Orbit Perigee
SMA = R_p/(1-ecc);                      % [km]          - Initial Orbit Semi-Major Axis
n = sqrt(mu./SMA.^3);                   % [rad/sec]     - Mean motion
eta = sqrt(1-ecc.^2);                   %               - Convenient eccentricity measure

%%%% Note:
% Separate parameters for combined maneuver, plot surface of di vs. dOM on
% a certain number of discrete orbit radiuses 
Discrete_Radius = [6800 7500 10000];

%%%% Note: 
% Inclination appears in equations, but isn't used. Here in case later
% version need to look at it
inc = 51.6*deg2rad;                     % [deg]         - Orbit Inclination (Constant)
%%%% Note:
% Change in RAAN appears in the equations, but we aren't concerned with it
% here. It is included as a zero matrice for transparency, or if later
% versions wish to use it as another parameter.
delta_OM_r = zeros(num_pts,num_pts);    % [deg]         - RAAN Change

% Parameterized variables we're interested in
delta_om_d = linspace(0,5,num_pts);     % [deg]         - Argument of Perigee change
delta_M_d = linspace(0,5,num_pts);      % [km]          - Mean anomaly change

% Convert to radians
delta_om_r = delta_om_d*deg2rad;
delta_M_r = delta_M_d*deg2rad;     

% Calculate the DeltaV available with the used thruster
% Ideal Rocket Equation, See Vallado Equation 6-42
Dv_avail = g*spacecraft.I_sp*log(spacecraft.M_total/(spacecraft.M_total-spacecraft.M_prop));

% Cases to consider
method = {'Arg. of Perigee Maneuver','Mean Anomaly Maneuver','Combined Maneuver'};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Building the DeltaV design space

% Equation for Arg. of Perigee/Mean Anomaly change
% See Schaub & Alfriend, Eq. 16 & 17
Dvp = @(del_om,del_M,a,nn,e) -(nn.*a/4).*((((1+e).^2)./eta).*(del_om + ...
    delta_OM_r*cos(inc)) + del_M);
Dva = @(del_om,del_M,a,nn,e) (nn.*a/4).*((((1-e).^2)./eta).*(del_om + ...
    delta_OM_r*cos(inc)) + del_M);

% Build matrices for orbital element changes
domMat = repmat(delta_om_r,num_pts,1);
dMMat = repmat(delta_M_r',1,num_pts);

% Memory allocation
Dv_Req = zeros(num_pts,num_pts,num_pts);
% Calculates each burn's DeltaV and sums them for the total burn DeltaV
for ii=1:num_pts
    Dv_Req(:,:,ii) = abs(Dvp(domMat,dMMat,SMA(ii),n(ii),ecc))+...
        abs(Dva(domMat,dMMat,SMA(ii),n(ii),ecc));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Calculate DeltaV & mass percent for each maneuver case

for iter=1:length(method)
    
    % Switch on maneuver case
    maneuver_case  = method{iter};
    switch maneuver_case
        case 'Arg. of Perigee Maneuver'
            display(method(iter))
            
            % Memory allocation & reset temporary variables
            Mass_percent = zeros(num_pts,num_pts);
            Dv_total = Dv_Req(1,:,:);
            
            % Loop calculates mass percent for each SMA
            for ii=1:num_pts
                
                % Total DeltaV required cannot exceed DeltaV available
                index = Dv_total(1,:,ii)>=Dv_avail;
                Dv_total(1,index,ii) = NaN;
                
                % Determine mass with Ideal Rocket Equation
                M_prop_vec = spacecraft.M_total*(1-exp(-(Dv_total(1,:,ii))/(g*spacecraft.I_sp)));
                Mass_percent(:,ii) = M_prop_vec/nominalPropMass*100;
            end
            
            % Plotting 
            figure(iter)
            hold on
            grid on
            surf(SMA,delta_om_d,Mass_percent,'EdgeColor','None')
            c = colorbar;
            c.Label.String = 'Percent Propellant Mass Burned';
            title1 = title(method(iter));
            xl = xlabel('Orbit Radius, [km]');
            yl = ylabel('Arg. of Perigee Change $\delta \omega$, [deg]');
            set([title1 xl yl],'interpreter','latex','fontsize',12)
            axis tight
            hold off
          
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 'Mean Anomaly Maneuver'
            display(method(iter))
            
            % Memory allocation & reset temporary variables
            Mass_percent = zeros(num_pts,num_pts);
            Dv_total = Dv_Req(:,1,:);
            
            % Loop calculates mass percent for each SMA
            for ii=1:num_pts
                
                % Total DeltaV required cannot exceed DeltaV available
                index = Dv_total(:,1,ii)>=Dv_avail;
                Dv_total(index,1,ii) = NaN;
                
                % Determine mass with Ideal Rocket Equation
                M_prop_vec = spacecraft.M_total*(1-exp(-(Dv_total(:,1,ii))/(g*spacecraft.I_sp)));
                Mass_percent(:,ii) = M_prop_vec/nominalPropMass*100;
            end
            
            % Plotting
            figure(iter)
            hold on
            grid on
            surf(SMA,delta_M_d,Mass_percent,'EdgeColor','None')
            c = colorbar;
            c.Label.String = 'Percent Propellant Mass Burned';
            title1 = title(method(iter));
            xl = xlabel('Orbit Radius, [km]');
            yl = ylabel('Mean Anomaly Change $\delta M$, [deg]');
            set([title1 xl yl],'interpreter','latex','fontsize',12)
            axis tight
            hold off
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 'Combined Maneuver'
            display(method(iter))
            
            % Determine the discrete SMA to calculate, get mean motion
            SMA = Discrete_Radius/(1-ecc);
            n = sqrt(mu./SMA.^3); 
            
            Dv_Req = zeros(num_pts,num_pts,length(SMA));
            % Recalculates DeltaV required at discrete SMA
            for ii=1:length(SMA)
                Dv_Req(:,:,ii) = abs(Dvp(domMat,dMMat,SMA(ii),n(ii),ecc)) + ...
                    abs(Dva(domMat,dMMat,SMA(ii),n(ii),ecc));
            end
            
            % Memory allocation & reset temporary variables
            Mass_percent = zeros(num_pts,num_pts);
            Dv_total = Dv_Req;
            
            % Loop calculates mass percent for each discrete SMA 
            for ii=1:length(SMA)
                
                % Delta V required cannot exceed Delta V available
                index = Dv_total(:,:,ii)>=Dv_avail;
                for kk=1:num_pts
                    for jj=1:num_pts
                        if Dv_total(kk,jj,ii)>=Dv_avail
                            Dv_total(kk,jj,ii) = NaN;
                        end
                    end
                end
                
                % Determine mass with Ideal Rocket Equation
                M_prop_vec = spacecraft.M_total*(1-exp(-(Dv_total(:,:,ii))/(g*spacecraft.I_sp)));
                Mass_percent(:,:,ii) = M_prop_vec/nominalPropMass*100;
                
                % Plotting
                figure(iter+ii-1)
                hold on
                grid on
                surf(delta_om_d,delta_M_d,Mass_percent(:,:,ii),'EdgeColor','None')
                c = colorbar;
                c.Label.String = 'Percent Propellant Mass Burned';
                title1 = title(strcat(method(iter), ', $a=',num2str(SMA(ii)),'$'));
                xl = xlabel('Arg. of Perigee Change $\delta \omega$, [deg]');
                yl = ylabel('Mean Anomaly Change $\delta M$, [deg]');
                set([title1 xl yl],'interpreter','latex','fontsize',12)
                axis tight
                hold off
            end         
    end

end

