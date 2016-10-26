%%%%% Configuration script for LERM Model

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
        B = [zeros(3,2); 
            1 0; 
            0 0; 
            0 1];
    case 3
        B = [0 0 0; 
             1 0 0; 
             0 0 0; 
             0 1 0; 
             0 0 0; 
             0 0 1];
end

eccFactor = -n*(2+ecc)/(sqrt((1+ecc)*(1-ecc)^3));

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
        RelInitState = [x0 xd0 y0 yd0 z0 zd0]';
end

% Final deputy orbit description
method = deputyOrbitDescriptionFinal;
switch method
    case 'Cartesian'
        RelFinalState = [xf xdf yf ydf zf zdf]';
end

% Create structure to intialize model class
initStruct.descriptor = 'LERM';
initStruct.params = {Req,mu,tol};
initStruct.maneuverParams = {samples,B,umax,umin};
initStruct.timeParams.t0 = t0;
initStruct.timeParams.dt = dt;
initStruct.timeParams.tf = tf;
initStruct.initChiefDescription = chiefOrbitDescription;
initStruct.RelInitState = RelInitState(:);
initStruct.RelFinalState = RelFinalState(:);
initStruct.Elements = Elements(:);