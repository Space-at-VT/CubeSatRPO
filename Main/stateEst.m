function [kalman ] = stateEst( rAct, vAct, u, kalman, t )
%stateEst Given the actual state, find the measured state, and run through 
%INPUTS:
% rAct -------- Actual relative position vector; [x; y; z];
% vAct -------- Actual relative velocity vector; [vx, vy, vz];
% thrust ------ Thrust 
% kalman ------ Struct containing variables for the filter
%        .ii ----- Loop counter 
%        .t ------ Time (vector length ii)
%        .xMeas -- Measured state (matrix 6xii)
%        .A ------ State Transition Matrix (matrix 6x6), function of time
%                  differential
%        .B ------ Control input model (NOT CURRENTLY IN USE) (matrix 6x6)
%        .u ------ Control vector u (NOT CURRENTLY IN USE) (array 6x1)
%        .xEst --- Estimated state from filter (matrix 6 x ii)
%        .P ------ State covariance matrix (matrix 6x6)
%        .Q ------ Process noise covariance (matrix 6x6)
%        .R ------ Measurement noise covariance (matrix 6x6)
%        .H ------ Measurement map matrix (matrix 6x6)

%% METHOD
% ASSUMPTION: Cheif Satellite Position, Velocity Vectors known in ECI
% ASSUMPTION: Cheif Satellite attitude known (can move from sensor
%             measurements to relative position in ECI)

% Given: relative position of cheif satellite from target satellite in RSW
% coordinates centered at the target satellite (For simulation)

% Given: relative velocity of cheif satellite from target satellite in RSW
% coordinates centered at the target satellite (For simulation)

% Assume: some arbitrary circular orbit, with semi-major axis 10e6 meters
% Use orbit to find ECI position, velocity of target satellite as function
% of time

% Use ECI position, velocity to find transformation from ECI to RSW

% Cheif can measure relative position in body frame
% Cheif attitude known; can find relative position in ECI frame

% Take three position measurements.  Use Gibb's method to determine
% measured orbit.

% From measured orbit, find velocity in ECI for current time

% Use measured position, velocity to find measured ECI -> RSW frame
% transformation

% Use transformation to find measured relative position in RSW

% Run measured relative position through filter, find estimated relative
% position in RSW

% Subtract estimated target velocity in ECI from cheif velocity in ECI, use
% transform to get into RSW frame



% Cheif Satellite position, relative position of target known in ECI: can
% find position of target satellite in ECI



% %% Target Orbital Parameters:
% mu = 3.9860004418*10^14;
% a = 10e6; %m - Semimajor axis
% e = 0; %Eccentricity, circular orbit
% i = 0; %No inclination
% rightAcencion = 0; %Right ascension
% argPerigee = 0; %Argument of 
% nu0 = 0; %True anomaly
% r = a; p = a; %Circular orbit
% TP = (2*pi*a^(3/2))/sqrt(mu);

% %% Orbit simulation
% 
% % Find Target actual position in ECI -------------------------------------
% nu = (t/TP)*360+nu0; %Update the true anomaly
% while nu > 360
%     nu = nu-360;
% end
% 
% targetRpqw = [r*cosd(nu); r*sind(nu); 0]; %m - R vector in PQW
% targetVpqw = sqrt(mu/p)*[-sind(nu); e + cosd(nu); 0]; %m/s - V vector in PQW
% 
% 
% %For orbital elements given, PQW frame is identical to ECI Frame
% targetReci = targetRpqw; %m
% kalman.targetVeci(:,ii) = targetVpqw; %m/s
% 
% 
% % Find RSW coordinate transformation
% [T,~,~] = ECI2RSW(targetReci,kalman.targetVeci(:,ii));
% 
% 
% % Find relative R and V in ECI coordinates
% Rdiff = T'*rAct;
% Vdiff = T'*vAct;
% 
% % Find cheif position, velocity in ECI
% chiefReci = targetReci+Rdiff;
% chiefVeci = kalman.targetVeci(:,ii)+Vdiff;
% 
% % Measure relative position
% kalman.RrelECI(:,ii) = measureState(chiefReci-targetReci);
% 
% % Find target position in ECI
% kalman.targetReci(:,ii) = chiefReci - kalman.RrelECI(:,ii);
% 
% % Need more measurements to find relative velocity.
% % TODO: Do something better with this.
% if ii<3
%     rEst = NaN;
%     vEst = NaN;
%     return;
% end
% 
% if ii == 3
%     gibbsOrbitDeterm(kalman.targetReci,kalman.t);
%     
% end



%% Filter Setup

%update loop counter
kalman.ii = kalman.ii+1;
ii = kalman.ii;


%Store actual state
kalman.xAct(:,ii) = [rAct; vAct];

%update current time
kalman.t(ii) = t;

%Get time differential;
tau = t - kalman.t(ii-1);

kalman.u(:,ii) = u; %Acceleration control vector

%Get measured state
kalman.xMeas(:,ii) = measureState( rAct);

%Predict new state.  Put in control later
xPred = kalman.A(tau)*kalman.xEst(:,ii-1) + kalman.B(tau)*kalman.u(:,ii);

%Predict estimate covariance
Ppred = kalman.A(tau)*kalman.P*kalman.A(tau)' + kalman.Q;

%Innovation or measurement residual
y = kalman.xMeas(:,ii) - kalman.H*xPred;

%Innovation covariance
S = kalman.H*Ppred*kalman.H' + kalman.R;

%Compute the gain
% K = (kalman.H*kalman.P(ii-1)*kalman.H' + kalman.R(ii))\(kalman.P(ii-1)*H');
% K = S\(Ppred*kalman.H'); %Ppred*H'*inv(S) is the same
K = Ppred*kalman.H'*inv(S);
%Update state prediction with measurements
kalman.xEst(:,ii) = xPred + K*y;

%Update state covariance
kalman.P = (eye(6) - K*kalman.H)*kalman.P;

rEst = kalman.xEst(1:3,ii);
vEst = kalman.xEst(4:6,ii);


end

