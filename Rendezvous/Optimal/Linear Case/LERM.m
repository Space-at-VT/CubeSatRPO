classdef LERM < handle
    properties
        % Maneuver parameters
        Phi                     % The LERM STM history
        X                       % Curvilinear state history
        Ak                      % Dynamics matrix at step k
        Bk                      % Input matrix at step k
        B                       % Continuous input matrix
        A
        
        % Scenario parameters
        Req                     % Radius of the Earth
        mu                      % Gravitational parameter of Earth
        tol                     % Tolerance for Kepler solver
        
        % Orbit descriptors
        ChiefElemsNSMean        % Mean, nonsingular description of Chief
        chiefOrbitDescription   % Descriptor flag for chief's orbit
        initialCondition        % Deputy initial conditions
        terminalCondition       % Deputy terminal conditions
        kepElemsInit            % Initial Kepler Elements of chief
        
        % Time parameters 
        t0                      % Initial time
        dt                      % Sample time
        tf                      % Final time
        time                    % Time vector
        
        samples
        numPeriod
        period
    end
    
    methods
        % Initialization of LERM Class
        function obj = LERM(initStruct)
            % Retrieving scenario parameters
            obj.Req             = initStruct.params{1};
            obj.mu              = initStruct.params{2};
            obj.tol             = initStruct.params{3};
            % Retrieving maneuver parameters
            obj.samples         = initStruct.maneuverParams{1};
            obj.B               = initStruct.maneuverParams{2};
            % Retrieving time parameters
            obj.t0              = initStruct.timeParams.t0;
            obj.dt              = initStruct.timeParams.dt;
            obj.tf              = initStruct.timeParams.tf; 
            
            % Retrieving chief orbit descriptor
            obj.chiefOrbitDescription = initStruct.initChiefDescription;
            method = obj.chiefOrbitDescription;
            % Retrieve Chief Orbital Elements
            switch method
                case 'Classical'
                    obj.kepElemsInit = initStruct.Elements;
                case 'Nonsingular'
                    % Convert to Classical orbital elements
                    obj.ChiefElemsNSMean = initStruct.Elements;
                    obj.kepElemsInit = Nonsingular_to_COE(obj.ChiefElemsNSMean);
            end
            
            % RetrieveDeputy Initial/Terminal states
            obj.initialCondition = initStruct.RelInitState;
            % Convert terminal conditions into final state
            if isempty(initStruct.RelFinalState);
                obj.terminalCondition = [];
            else
                obj.terminalCondition = initStruct.RelFinalState;
            end
            % Create time vector with time parameters
            obj.makeTimeVector();
        end
        
        function obj = makeTimeVector(obj)
            % Creates the appropriate time vector 
            method = obj.chiefOrbitDescription;
            switch method
                case 'Classical'
                    n = sqrt(obj.mu/obj.kepElemsInit(1)^3);
                    if isempty(obj.t0)
                        obj.t0 = 0;
                    else
                    end
                    obj.period = 2*pi/n;
                    obj.time = obj.t0:obj.dt:obj.tf;
                    
                case 'Nonsingular'
                    n = sqrt(obj.mu/obj.ChiefElemsNSMean(1)^3);
                    if isempty(obj.t0)
                        obj.t0 = 0;
                    else
                    end
                    obj.period = 2*pi/n;
                    obj.time = obj.t0:obj.dt:obj.tf;
            end
        end
        
        % Propogate model forwards in time
        function obj = propagateModel(obj,t1,t2)
            if nargin < 2
                t1 = obj.t0;
            end
            if nargin < 3
                t2 = obj.tf;
            end
            t = t1:obj.dt:t2;
            obj.Phi = BrouckeSTM(t,obj.mu,obj.kepElemsInit,obj.tol);
        end
        
%         function obj = propagateModel(obj)
%             obj.A = LERM_Matrix(t(ii),obj.mu,obj.kepElemsInit,obj.tol);
%             Ad = expm(obj.A*obj.dt);
%             for ii = 1:length(obj.time)-1
%                 obj.Phi(:,:,ii) = Ad^ii;
%             end
%         end
        
        % Propogate the state vector forwards in time with propagated model
        function obj = propagateState(obj)
            obj.propagateModel();
            for ii = 1:length(obj.time)
                obj.X(:,ii) = obj.Phi(:,:,ii)*obj.initialCondition;
            end
        end

%         function obj = propagateState(obj)
%             obj.propagateModel();
%             obj.X = zeros(6, length(obj.time));
%             obj.X(:,1) = obj.initialCondition;
%             for ii = 1:length(obj.time)-1
%                 obj.X(:,ii+1) = obj.Phi(:,:,ii)*obj.initialCondition;
%             end
%         end

        function obj = makeDiscreteMatrices(obj)
            t = obj.time;
            obj.Ak = zeros(6,6,length(t));
            obj.Bk = zeros(6,size(obj.B,2),length(t));
            for ii=1:length(t)
                obj.A = LERM_Matrix(t(ii),obj.mu,obj.kepElemsInit,obj.tol);
                sysc = ss(obj.A,obj.B,[],[]);
                sysd = c2d(sysc,obj.dt,'zoh');
                obj.Ak(:,:,ii) = sysd.A;
                obj.Bk(:,:,ii) = sysd.B;
            end
            
        end

%         function obj = makeDiscreteMatrices(obj)
%             t = obj.time;
%             N = length(t);
% %             obj.propagateModel();
% %             obj.Ak = obj.Phi;
%             for k = 1:N
%                 % Create sub-intervals of time with samples
%                 Tk = linspace(k*obj.dt,(k+1)*obj.dt,obj.samples);
%                 % Get GA-STM at each time in sub-interval
%                 Phi_k = BrouckeSTM(Tk,obj.mu,obj.kepElemsInit,obj.tol);
%                 % Discrete dynamics matrix is the final STM
%                 phiend = Phi_k(:,:,end); % Phi(k+1,k)
%                 
%                 % Get "B" transition matrices from each subinterval to the
%                 % next timestep
%                 % Phi(k+1,k+Tk(i)) = Phi(k+1,k)*Phi(k,k+Tk(i))*B(t)
%                 % Phi(k,k+Tk(i)) = Phi(k+Tk(i),k)^-1
%                 obj.Ak(:,:,k) = phiend;
%                 for ii = 1:length(Tk)
%                     PhiBk(:,:,ii) = phiend*Phi_k(:,:,ii)^(-1)*obj.B;
%                 end
%                 
%                 % Sum the average transition matrices btwn subintervals
%                 Bd = zeros(size(obj.B));
%                 for ii = 1:length(Tk) - 1
%                     Bd = Bd + 1/2.*(Tk(ii+1)-Tk(ii))*(PhiBk(:,:,ii+1) + PhiBk(:,:,ii));
%                 end
%                 % The fully discretized input matrix
%                 obj.Bk(:,:,k) = Bd;
%             end
%         end

    end
end

function Phi = BrouckeSTM(tspan,mu,ChiefKeplerElem,tol)
% Author: Dylan Thomas
%
% Function to implement State-Transition Matrix (STM) for the Linear
% Equations of Relative Motion (LERM) in continuous-time (Broucke, 2003).
% Allows one to propogate relative state vector in LVLH frame forward in
% time without numeric integration.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inputs:
%           tspan
%           mu
%           
% Outputs:
%
%
%
%
TT = [1 0 0 0 0 0;
      0 0 1 0 0 0;
      0 1 0 0 0 0;
      0 0 0 1 0 0;
      0 0 0 0 1 0;
      0 0 0 0 0 1];
% Retrieive needed orbital elements
a       = ChiefKeplerElem(1);
ecc     = ChiefKeplerElem(2);
omega   = ChiefKeplerElem(5);
% Calculate convenient constansts
n       = sqrt(mu/a^3);
eta     = sqrt(1-ecc^2);
p       = a*(1-ecc^2);
% Determine Fundamental Matrix Inverse at initial time
t0                  = tspan(1);
[r0,~,theta,~,~,E0] = ffcoef(t0,mu,a,ecc,omega,tol);
f0                  = theta - omega;
[RIxy,RIz]          = RInverseMatrix(t0,mu,a,ecc,r0,f0,E0,n,eta,p);
% RI = [RIxy zeros(4,2); zeros(2,4) RIz];
% Initialize STM
Phi = zeros(6,6,length(tspan));
for ii = 1:length(tspan)
    % Determine change in time from initial to current time
    dt                      = tspan(ii)-t0;
    % Retrieve coefficients at current time
    [r,~,theta,~,~,EccAnom] = ffcoef(dt,mu,a,ecc,omega,tol);
    f                       = theta - omega;
    % Calculate Fundamental Matrix at current time
    [Rxy,Rz]                = Rmatrix(dt,mu,a,ecc,r,f,EccAnom,n,eta,p);
    % Get the in-plane & out-of-plane STM's
    Phixy                   = Rxy*RIxy;
    Phiz                    = Rz*RIz;
    % Concatenate into full STM
    Phi(:,:,ii)             = TT*[Phixy zeros(4,2); zeros(2,4) Phiz]*TT;
%     R = [Rxy zeros(4,2); zeros(2,4) Rz];
%     Phi(:,:,ii) = R*RI;

end
end

function [Rxy,Rz] = Rmatrix(dt,mu,a,ecc,r,f,EccAnom,n,eta,p)
% Calculates the Fundamental Matrices at a time dt from t_0

% Special condition for circular orbits
if ecc == 0
    M = EccAnom;
    % In-plane Fundamental Matrix
    Rxy = [1            -cos(M)     sin(M)      0;
           -1.5*n*dt  2*sin(M)    2*cos(M)    1;
           0            n*sin(M)    n*cos(M)    0;
           -1.5*n     2*n*cos(M)  -2*n*sin(M) 0];
else
    % In-plane Fundamental Matrix columns
    Rxy(:,1) = [(r/a) - 1.5*(n*dt*ecc*sin(f))/(eta);
                -1.5*n*dt*eta*(a/r);
                -(n*ecc*sin(f))/(2*eta) - 1.5*dt*ecc*cos(f)*((n*a)/r)^2;
                1.5*dt*ecc*sin(f)*((n*a)/r)^2 - 1.5*eta*((n*a)/r)];
    
    Rxy(:,2) = [-cos(f);
                (1+r/p)*sin(f);
                n*sin(f)*eta*(a/r)^2
                n*eta*(1+r/p)*(a/r)^2*cos(f) + (ecc*n*(sin(f))^2)/sqrt((1-ecc^2)^3)];
    
    Rxy(:,3) = [ecc*sin(f)/eta;
                eta*(a/r);
                ecc*n*cos(f)*(a/r)^2;
                -ecc*n*sin(f)*(a/r)^2];
    
    Rxy(:,4) = [0;
                r/a;
                0;
                (ecc*n*sin(f))/eta];
end
% Components for the out-of-plane solution
z1      = sqrt(a*n/mu)*r*cos(f); 
z2      = sqrt(a*n/(mu*(1-ecc^2)))*r*sin(f);
z1Dot   = -a*sqrt(n)*sin(EccAnom)/r; z2Dot = a*sqrt(n)*cos(EccAnom)/r;
% Create out-of plane fundamental matrix
Rz      = [z1, z2; z1Dot, z2Dot];
end

function [RIxy,RIz] = RInverseMatrix(dt,mu,a,ecc,r,f,EccAnom,n,eta,p)
% Calculates the inverse Fundamental Matrices at a time dt from t_0

% Special condition for circular orbits
if ecc == 0
    M = EccAnom;
    % In-plane Inverse Fundamental Matrix
    RIxy = [4           0 0         2/n;
            3*cos(M)    0 sin(M)/n  2*cos(M)/n;
            -3*sin(M)   0 cos(M)/n  -2*sin(M)/n;
            6*n*dt      1 -2/n      3*dt];
else
    % Sub-matrices which compose the full inverse fundamental matrix
    Imatrix = [2*(a/r)^2*(1+p/r),       (a/r)*(3*cos(f) + ecc*(2+(cos(f))^2));
                -2*(a/r)^2*ecc*sin(f),  -ecc*sin(f)*(a/p)*(ecc+cos(f)*(2+ecc*cos(f)));
               ((2*ecc*sin(f))/(n*eta)), (eta/n)*sin(f);
                2*(a/r)*(eta/n),        (eta/n)*(cos(f)+(r/p)*(ecc+cos(f)))];
    
    Jmatrix = [(-a/p)*sin(f)*(ecc^2+(1+ecc*cos(f))*(3+ecc*cos(f))), -(a^2/(p*r))*ecc*sin(f)*(2+(p/r)+(r/p));
                (a/p)*ecc*(sin(f))^2*(2+ecc*cos(f)),                  (a/p)^2*(1+ecc*cos(f) - ecc^2*(ecc*(cos(f))^3+2*(cos(f))^2-1));
                ((r*eta)/(p*n))*(cos(f)+ecc*((cos(f))^2-2)),            ((ecc*cos(f)-1)/(n*eta))*(1+(r/p));
                -(eta/n)*sin(f)*(1+(r/p)),                          -((ecc*sin(f))/(n*eta))*(1+(r/p))];
    
    Kmatrix = [((n*ecc)/eta)*(a/r)*(1+(p/r)),   (n/sqrt((1-ecc^2)^3))*(a/r)^2*(1+(p/r));
                -((n*ecc^2*sin(f))/eta)*(a/r)^2, -((n*ecc*sin(f))/sqrt((1-ecc^2)^3))*(a/r)^2;
                (ecc^2*sin(f))/(1-ecc^2),       (ecc*sin(f))/((1-ecc^2)^2);
                (a*ecc)/r,                      a/((1-ecc^2)*r)];
    % In-plane Fundamental Matrix
    RIxy = transpose([Imatrix, Jmatrix + 3*dt*Kmatrix]);
end
% Components for the out-of-plane solution
z1      = sqrt(a*n/mu)*r*cos(f); 
z2      = sqrt(a*n/(mu*(1-ecc^2)))*r*sin(f);
z1Dot   = -a*sqrt(n)*sin(EccAnom)/r; z2Dot = a*sqrt(n)*cos(EccAnom)/r;
% Create out-of plane fundamental matrix
RIz     = [z2Dot, -z2; -z1Dot, z1];

end

function [r,rdot,theta,thetadot,thetaddot,EccAnom] = ffcoef(dt,mu,a,ecc,omega,tol)
% Time-varying Formation Flying coefficients derived by parameterizing the 
% orbit equation in polar form and treat the problem as planar. 

% Get constants
n           = sqrt(mu/a^3);
MeanAnom    = n*dt;
[EccAnom,f] = kepler(MeanAnom,ecc,tol);
theta       = omega + f;

% Equations for coefficients
r           = a*(1-ecc^2)/(1+ecc*cos(theta));
rdot        = sqrt(mu*a*(1-ecc^2))*(1+ecc*cos(theta))^2/(a^2*(1-ecc^2)^2);
thetadot    = ecc*mu*sin(theta)/(sqrt(mu*a*(1-ecc^2)));
thetaddot   = 2*mu*ecc*(1+ecc*cos(theta))^3*sin(theta)/(a^3*(1-ecc^2)^3);
end

%%% Save function for later?
function A = LERM_Matrix(t,mu,ChiefKeplerElem,tol)
% Time varying coefficients and solution to Kepler's equation
a       = ChiefKeplerElem(1);
ecc     = ChiefKeplerElem(2);
n = sqrt(mu/a^3);
M = n.*t;
M = M(:);
[~,f] = kepler(M,ecc,tol);
r = (a.*(1-ecc.^2))./(1+ecc.*cos(f));
fdot = sqrt(mu.*a.*(1-ecc.^2)).*(1+ecc.*cos(f)).^2./(a.^2.*(1-ecc.^2).^2);
rdot = ecc.*sin(f).*sqrt(mu.*a.*(1-ecc.^2))./(a.*(1-ecc.^2));
fddot = -2.*rdot.*fdot./r;

% State matrix for LERM
AA = [0,0,0,1,0,0;
      0,0,0,0,1,0;
      0,0,0,0,0,1;
      (fdot^2+2*mu/r^3) fddot 0 0 2*fdot 0;
      -fddot (fdot^2-mu/r^3) 0 -2*fdot 0 0
      0 0 -mu/r^3 0 0 0];

TT = [1 0 0 0 0 0;
      0 0 0 1 0 0;
      0 1 0 0 0 0;
      0 0 0 0 1 0;
      0 0 1 0 0 0;
      0 0 0 0 0 1];
A = inv(TT)*AA*TT;
end

function kepElems = Nonsingular_to_COE(nsElems)
kepElems(1) = nsElems(1);
kepElems(2) = sqrt(nsElems(4)^2 + nsElems(5)^2);
kepElems(3) = nsElems(3);
kepElems(4) = nsElems(6);
kepElems(5) = acos(nsElems(4)/kepElems(2));
kepElems(6) = nsElems(2) - kepElems(5);
end

function [E, f] = kepler(M, e, Tol1)

% Kepler's Equation
E = M;
FF = 1;
while abs(FF) > Tol1
    FF = M - (E - e*sin(E));
    dFFdE = -(1 - e*cos(E));
    del_E = -FF / dFFdE;
    E = E + del_E;
end
while (E < 0)
    E = E + (2*pi);
end
while (E >= (2*pi))
    E = E - (2*pi);
end

%
kk_plus = 0;
while (M < 0)
    kk_plus = kk_plus + 1;
    M = M + (2*pi);
end
kk_minus = 0;
while (M >= (2*pi))
    kk_minus = kk_minus + 1;
    M = M - (2*pi);
end

% True Anomaly
f = 2*atan(sqrt((1+e)/(1-e))*tan(E/2));
if (E >= 0) && (E <= (pi))
    f = abs(f);
else
    f = (2*pi) - abs(f);
end;

f = f - kk_plus*(2*pi) + kk_minus*(2*pi);

end

