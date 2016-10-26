%%%%% Configuration script for HCW Model

% J2 coefficient
if strcmp(J2flag,'off') == 1
    J2  = 0; % Un-perturbed motion
elseif strcmp(J2flag,'on') == 1
    J2  = 1082.629e-6; % Perturbed motion
else
    error('Input error: Incorrect J2 Flag')
end

% Determine number of control directions & B matrix
switch numControlInput
    case 2
        B = [zeros(4,2); 
            eye(2)];
    case 3
        B = [zeros(3,3); 
            eye(3)];
end

eccFactor = -2*n;

% Bounded motion
if strcmp(BndMotionFlag,'True') == 1
    yd0 = eccFactor*x0; 
    ydf = eccFactor*xf;
elseif strcmp(BndMotionFlag,'False') == 1
    nothing();
else
    error('Input error: Incorrect Bounded Motion Flag')
end

% Initial deputy orbit description
method = deputyOrbitDescriptionInit;
switch method
    case 'Cartesian'
        RelInitState = [x0 y0 z0 xd0 yd0 zd0]';
end

% Final deputy orbit description
method = deputyOrbitDescriptionFinal;
switch method
    case 'Cartesian'
        RelFinalState = [xf yf zf xdf ydf zdf]';
end

% Create struture to initialize model class
initStruct.descriptor = 'HCW';
initStruct.params = {mu,a,RelInitState};
initStruct.terminalCondition = RelFinalState;
initStruct.timeParams.t0 = t0;
initStruct.timeParams.dt = dt;
initStruct.timeParams.tf = tf;
initStruct.maneuverParams = {samples,B,umax,umin};