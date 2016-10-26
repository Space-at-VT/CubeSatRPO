%%%%% Initialization for Optimal Control Problems

%% Toggleable variables
% J2flag = 'off';
J2flag = 'on';

% numControlInput = 2;
numControlInput = 3;

BndMotionFlag = 'True';
% BndMotionFlag = 'False';

%%%% Determine parameterizations to use
% Valid descriptions are 'Classical'; 'Nonsingular'
chiefOrbitDescription = 'Classical';
% Valid descriptions are 'Cartesian';
deputyOrbitDescriptionInit = 'Cartesian';
% Valid descriptions are 'Cartesian'; 
deputyOrbitDescriptionFinal = 'Cartesian';

% Check validity of orbit descriptions
[chiefOrbitDescription,deputyOrbitDescriptionInit,deputyOrbitDescriptionFinal]...
    = checkDescriptors(chiefOrbitDescription,deputyOrbitDescriptionInit,deputyOrbitDescriptionFinal);

%% Orbit Definitions: User-Defined

% Reference orbit/Chief description
method = chiefOrbitDescription;
switch method
    case 'Classical'
        a = 7000e3;
        ecc = 0.05;
        inc = 28*pi/180;
        raan = 0;
        w = 0;
        M0 = 0;
        Elements = [a ecc inc raan w M0]';
    case 'Nonsingular'
        a = 6678e3;
        th = 0;
        inc = 28*pi/180;
        q1 = 0;
        q2 = 0;
        raan = 45.006*pi/180;
        Elements = [a th inc q1 q2 raan]';
        ecc = sqrt(q1^2 + q2^2);
end
% Extra params
n = sqrt(mu/a^3);
period = 2*pi/n;

% Initial deputy orbit
method = deputyOrbitDescriptionInit;
switch method
    case 'Cartesian'
        x0 = 100;
        y0 = -400;
        z0 = 200;
        xd0 = -0.3;
        yd0 = 0;
        zd0 = 0;
    case 'Relative Classical'
        da = 200e3;
        de = 0;
        di = 0;
        dO = 0;
        dw = 0;
        dM = 0*pi/180;
    case 'Relative Nonsingular'
        da = -103.624;
        dth = -1.104e-3;
        di = 7.7076e-4;
        dq1 = 4.262e-5;
        dq2 = -9.708e-6;
        dO = 3.227e-3;
end

% Final deputy orbit
method = deputyOrbitDescriptionFinal;
switch method
    case 'Cartesian'
        xf = 10;
        yf = -1;
        zf = 0;
        xdf = 0;
        ydf = 0;
        zdf = 0;
    case 'Relative Classical'
        da = 0;
        de = 0;
        di = 0;
        dO = 0;
        dw = 0;
        dM = 0*pi/180;
    case 'Relative Nonsingular'
        da = 10000;
        dth = -1.104e-3;
        di = 7.7076e-4;
        dq1 = 4.262e-5;
        dq2 = -9.708e-6;
        dO = 3.227e-3;
end

%% Other User-defined variables
% tolerance for transcendental root finding
tol = 1e-13; 

% Number of time samples between each time step for discretization
samples = 3;

% Maximum Thrust Inputs
umax = 2.79*8/15;
umin = -umax;

% Time variables
numPeriod = 1;
t0 = 0;
dt = 1;
tf = 300;
% tf = period*numPeriod;