function [ kalman ] = kalmanSetup(rInit, vInit, p, q, r )
%kalmanSetup Sets up the initial values for the kalman filter

%% SETUP FOR 3 VARIABLES IN STATE

% mu = 3.9860004418*10^14;
% a = 10e6; %m - Semimajor axis
% e = 0; %Eccentricity, circular orbit
% i = 0; %No inclination
% rightAcencion = 0; %Right ascension
% argPerigee = 0; %Argument of 
% nu = 0; %True anomaly
% r = a; p = a; %Circular orbit
% 
% targetRpqw = [r*cosd(nu); r*sind(nu); 0]; %m - R vector in PQW
% targetVpqw = sqrt(mu/p)*[-sind(nu); e + cosd(nu); 0]; %m/s - V vector in PQW
% 
% 
% %For orbital elements given, PQW frame is identical to ECI Frame
% targetReci = targetRpqw; %m
% kalman.targetVeci(:,1) = targetVpqw; %m/s
% 
% % Find RSW coordinate transformation
% [T,~,~] = ECI2RSW(targetReci,kalman.targetVeci(:,1));
% 
% % Find relative R and V in ECI coordinates
% Rdiff = T'*rInit;
% Vdiff = T'*vInit;
% 
% % Find cheif position, velocity in ECI
% chiefReci = targetReci+Rdiff;
% chiefVeci = kalman.targetVeci(:,1)+Vdiff;
% 
% % Measure relative position
% kalman.RrelECI(:,1) = measureState(chiefReci-targetReci);
% 
% % Find target position in ECI
% kalman.targetReci(:,1) = chiefReci - kalman.RrelECI(:,1);
% % Target Velocity in ECI for testing purposes
% kalman.targetVeci(:,1)


%Initial setup command:
%kalman = kalmanSetup([x;y;z], [vx;vy;vz] );

% %State transition matrix
% kalman.A = @(tau) eye(3);
% 
% %Measurement mapping; for now, measures state without translation
% kalman.H = eye(3);
% 
% %Control mapping; thrust is along x vector?
% kalman.B = zeros(3);
% % kalman.B(4,4) = 1;
% kalman.u = @(tau) [0; 0; 0]; 
% 
% %Covariance Matrix of State Estimate
% kalman.P = eye(3) * 0.00001; %Estimated variance
% 
% %Process noise covariance
% kalman.Q = eye(3)*0.00001; %Estimated process noise
% 
% %Measurement noise covariance
% kalman.R = eye(3) * 0.00001;
% 
% %Initial measured state is taken as true;
% kalman.xMeas(:,1) = measureState(rInit);
% kalman.xEst(:,1) = kalman.xMeas(:,1);
% 
% %Initial time is 0
% kalman.t(1) = 0;
% 
% %loop counter
% kalman.ii = 1;
% 
% 
% 
% 

%% SETUP FOR 6 VARIABLES IN STATE
%Initial setup command:
%kalman = kalmanSetup([x;y;z], [vx;vy;vz] );

if (nargin == 2)
    p = 0.0001;
    q = 0.01;
    r = 0.1;
end

%Store actual state
kalman.xAct(:,1) = [rInit; vInit];

%State transition matrix
kalman.A = @(tau) [eye(3) eye(3)*tau;
                   zeros(3) eye(3)];

%Measurement mapping; for now, measures state without translation
kalman.H = [eye(3) zeros(3)];

%Control mapping; thrust is along x vector?
kalman.B = @(tau) [zeros(3); tau*eye(3)];
kalman.u = zeros(3,1); 

%Covariance Matrix of State Estimate
kalman.P = eye(6) * p; %Estimated variance

%Process noise covariance
kalman.Q = eye(6) * q; %Estimated process noise

%Measurement noise covariance
kalman.R = eye(3) * r;

%Initial measured state is taken as true;
kalman.xMeas(:,1) = measureState( rInit);
kalman.xEst(:,1) = [kalman.xMeas(:,1); 0;0;0];

%Initial time is 0
kalman.t(1) = -1;

%loop counter
kalman.ii = 1;

kalman.u(:,1) = [0;0;0];

end

