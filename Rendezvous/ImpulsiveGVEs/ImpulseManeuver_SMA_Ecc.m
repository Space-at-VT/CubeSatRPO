%%% Impulsive SMA and/or Eccentricity Maneuver
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

% Setting up vectorized parameters
ecc = 0;                                %               - Orbit Eccentricity 
init_alt = linspace(300,5000,num_pts);  % [km]          - Initial Orbit Altitude
R_p = init_alt + R_eq;                  % [km]          - Initial Orbit Perigee
SMA = R_p/(1-ecc);                      % [km]          - Initial Orbit Semi-Major Axis
n = sqrt(mu./SMA.^3);                   % [rad/sec]     - Mean motion
eta = sqrt(1-ecc.^2);                   %               - Convenient eccentricity measure

%%%% Note:
% Separate parameters for combined maneuver, plot surface of dSMA vs. de on
% a certain number of discrete orbit radiuses 
Discrete_Radius = [6800 7500 10000];

% Parameterized variables we're interested in
delta_ecc = linspace(0,0.5-ecc,num_pts); %               - Eccentricity Change
delta_SMA = linspace(0,1000,num_pts);    % [km]          - SMA Change

% Calculate the DeltaV available with the used thruster
% Ideal Rocket Equation, See Vallado Equation 6-42
Dv_avail = g*spacecraft.I_sp*log(spacecraft.M_total/(spacecraft.M_total-spacecraft.M_prop));

% Cases to consider
method = {'Semi-Major Axis Maneuver','Eccentricity Maneuver','Combined Maneuver'};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Building the DeltaV design space

% Equation for SMA/Eccentricity change
% See Schaub & Alfriend, Eq. 24 & 25
Dvp = @(del_a,del_ecc,a,nn,ecc) ((nn.*a.*eta)/4).*((del_a./a + del_ecc./(1+ecc)));
Dva = @(del_a,del_ecc,a,nn,ecc) ((nn.*a.*eta)/4).*((del_a./a - del_ecc./(1-ecc)));

% Build matrices for orbital element changes
dSMAMat = repmat(delta_SMA,num_pts,1);
deccMat = repmat(delta_ecc',1,num_pts);

% Memory allocation
Dv_Req = zeros(num_pts,num_pts,num_pts);
% Calculates each burn's DeltaV and sums them for the total burn DeltaV
for ii=1:num_pts
    Dv_Req(:,:,ii) = abs(Dvp(dSMAMat,deccMat,SMA(ii),n(ii),ecc))+...
        abs(Dva(dSMAMat,deccMat,SMA(ii),n(ii),ecc));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Calculate DeltaV & mass percent for each maneuver case

for iter=1:length(method)
    
    % Switch on maneuver case
    maneuver_case  = method{iter};
    switch maneuver_case
        case 'Semi-Major Axis Maneuver'
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
            surf(SMA,delta_SMA,Mass_percent,'EdgeColor','None')
            c = colorbar;
            c.Label.String = 'Percent Propellant Mass Burned';
            title1 = title(method(iter));
            xl = xlabel('Orbit Radius, [km]');
            yl = ylabel('Semi-Major Axis Change $\delta a$, [km]');
            set([title1 xl yl],'interpreter','latex','fontsize',12)
            axis tight
            hold off
          
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 'Eccentricity Maneuver'
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
            surf(SMA,delta_ecc,Mass_percent,'EdgeColor','None')
            c = colorbar;
            c.Label.String = 'Percent Propellant Mass Burned';
            title1 = title(method(iter));
            xl = xlabel('Orbit Radius, [km]');
            yl = ylabel('Eccentricity Change $\delta e$');
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
                Dv_Req(:,:,ii) = abs(Dvp(dSMAMat,deccMat,SMA(ii),n(ii),ecc))+...
                    abs(Dva(dSMAMat,deccMat,SMA(ii),n(ii),ecc));
            end
            
            % Memory allocation & reset temporary variables
            Mass_percent = zeros(num_pts,num_pts);
            Dv_total = Dv_Req;
            
            % Loop calculates mass percent for each radius
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
                surf(delta_SMA,delta_ecc,Mass_percent(:,:,ii),'EdgeColor','None')
                c = colorbar;
                c.Label.String = 'Percent Propellant Mass Burned';
                title1 = title(strcat(method(iter), ', $a=',num2str(SMA(ii)),'$'));
                xl = xlabel('Eccentricity Change $\delta e$');
                yl = ylabel('Semi-Major Axis Change $\delta a$, [km]');
                set([title1 xl yl],'interpreter','latex','fontsize',12)
                axis tight
                hold off
            end         
    end

end

