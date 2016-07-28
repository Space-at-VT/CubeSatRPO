%% Impulsive SMA and/or Eccentricity Maneuver
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
num_pts = 300;                          %               - Num. of Plot points
m_prop_nom = 1.5;                       % [kg]          - Nominal Propellant Mass
ecc = 0;                              %               - Eccentricity

% Setting up vectorized parameters
init_alt = linspace(300,5000,num_pts);  % [km]          - Initial Orbit Altitude
delta_ecc = linspace(0,0.6-ecc,num_pts);    %               - Change in Ecc
delta_SMA = linspace(0,1000,num_pts);    % [km]          - SMA Change
R_p = init_alt + R_e;                   % [km]          - Initial Orbit 
SMA_i = R_p/(1-ecc);
period = 2*pi*sqrt(SMA_i.^3/mu);          % [sec]         - Final Orbital period

% Intermediate parameters for easier reading
n = sqrt(mu./SMA_i.^3);               %               - Mean motion
eta = sqrt(1-ecc^2);

% Separate parameters for combined maneuver, plot surface of di vs. dOM on
% a certain number of discrete orbit radiuses 
Discrete_Radius = [6800 7500 10000];

% Cases to consider
method = {'Semi-Major Axis Maneuver','Eccentricity Maneuver','Combined Maneuver'};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Building the design space

% Equation for Arg. of Perigee/Mean Anomaly change
Dvp = @(del_a,del_ecc,a,nn,ecc) ((nn.*a.*eta)/4).*((del_a./a + del_ecc./(1+ecc)));
Dva = @(del_a,del_ecc,a,nn,ecc) ((nn.*a.*eta)/4).*((del_a./a - del_ecc./(1-ecc)));

% Build matrices for orbital element changes
dSMAMat = repmat(delta_SMA,num_pts,1);
deccMat = repmat(delta_ecc',1,num_pts);

% Memory allocation
Dv_Req = zeros(num_pts,num_pts,num_pts);
tic
for ii=1:num_pts
    Dv_Req(:,:,ii) = abs(Dvp(dSMAMat,deccMat,SMA_i(ii),n(ii),ecc))+...
        abs(Dva(dSMAMat,deccMat,SMA_i(ii),n(ii),ecc));
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
        case 'Semi-Major Axis Maneuver'
            display(method(iter))
            
            % Equation for thrust
            a_thrusti(iter) = spacecraft.T_nom/spacecraft.M_total;
            
            % Find total DeltaV & Time of Flight
            Dv_avail(iter) = g*spacecraft.I_sp*log(spacecraft.M_total/(spacecraft.M_total-spacecraft.M_prop));
            Tburn(iter) = (Dv_avail(iter)*1000)/(a_thrusti(iter));
            
            % Memory allocation & reset temporary variables
            Mass_percent = zeros(num_pts,num_pts);
            Dv_total = Dv_Req(1,:,:);
            
            % Loop calculates mass percent for each radius
            for ii=1:num_pts
                
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
            surf(SMA_i,delta_SMA,Mass_percent,'EdgeColor','None')
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
            
            % Equation for thrust
            a_thrusti(iter) = spacecraft.T_nom/spacecraft.M_total;
            
            % Find total DeltaV & Time of Flight
            Dv_avail(iter) = g*spacecraft.I_sp*log(spacecraft.M_total/(spacecraft.M_total-spacecraft.M_prop));
            Tburn(iter) = (Dv_avail(iter)*1000)/(a_thrusti(iter));
            
            % Memory allocation & reset temporary variables
            Mass_percent = zeros(num_pts,num_pts);
            Dv_total = Dv_Req(:,1,:);
            
            % Loop calculates mass percent for each radius
            for ii=1:num_pts
                
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
            surf(SMA_i,delta_ecc,Mass_percent,'EdgeColor','None')
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
            
            %%% TODO: Figure this out
            SMA_i = Discrete_Radius/(1-ecc);
            n = (mu./SMA_i.^3).^0.5;
            period = 2*pi*sqrt(SMA_i.^3/mu);
            Dv_Req = zeros(num_pts,num_pts,length(SMA_i));
            for ii=1:length(SMA_i)
                Dv_Req(:,:,ii) = abs(Dvp(dSMAMat,deccMat,SMA_i(ii),n(ii),ecc))+...
                    abs(Dva(dSMAMat,deccMat,SMA_i(ii),n(ii),ecc));
            end
            
            % Equations for thrust
            a_thrusti(iter) = spacecraft.T_nom/spacecraft.M_total;
            
            % Find total DeltaV & Time of Flight
            Dv_avail(iter) = g*spacecraft.I_sp*log(spacecraft.M_total/(spacecraft.M_total-spacecraft.M_prop));
            Tburn(iter) = (Dv_avail(iter)*1000)/(a_thrusti(iter));
            
            % Memory allocation & reset temporary variables
            Mass_percent = zeros(num_pts,num_pts);
            Dv_total = Dv_Req;
            
            % Loop calculates mass percent for each radius
            for ii=1:length(SMA_i)
                
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
                surf(delta_SMA,delta_ecc,Mass_percent(:,:,ii),'EdgeColor','None')
                c = colorbar;
                c.Label.String = 'Percent Propellant Mass Burned';
                title1 = title(strcat(method(iter), ', $R=',num2str(SMA_i(ii)),'$'));
                xl = xlabel('Eccentricity Change $\delta e$');
                yl = ylabel('Semi-Major Axis Change $\delta a$, [km]');
                set([title1 xl yl],'interpreter','latex','fontsize',12)
                axis tight
                hold off
            end         
            
    end

end

