% Translational trajcetory generation is all the same as the summer.
% Organized to be more easily commanded. All attitude modeling is still in
% development. Basic damping and PD in included for perturbations caused
% from the off-centering of thrusters. As of 9/8 the model is not coupled
% with translational movement. The linearity of the model may limit the
% accuracy of coupling rotation with translation.

classdef newSatellite
    %satellite defines the properties of a satellite as well as its trajectory
    % relative to the origin
    
    % Default properties of a basic cubesat
    properties
        %% Graphics
        name = 'CubeSat'            %Satellite name
        EOM = 'HCW'                 %Relative motion model
        mode = 'approach'           %Satellite objective
        color = 'b'                 %Graph color
        
        %% Satellite parameters
        umax = 0.25                 %Thrust,                N
        ISP = 150                   %Specific impulse,      s
        dryMass = 10                %Dry mass,              kg
        fuel = 0.5                  %Fuel mass,             kg
        vmax = 0.5                  %Max velocity,          m/s
        bnd = [0.1,0.1,0.1]         %Satellite size,        m
        Tmax = 0.25                 %Max reaction torque
        kp = 0.1                    %Position damping
        kd = 0.7                    %Velocity damping
        ki = 0.3                    %Integral damping
        dx = 0.01                   %x-axis moment arm,     m
        dy = 0.01                   %y-axis moment arm,     m
        dz = 0.01                   %z-axis moment arm,     m
               
        %% Trajectory
        x = 0                       %x position over time,  m
        y = 0                       %y position over time,  m
        z = 0                       %z position over time,  m
        vx = 0                      %Velocity over time,    m/s
        vy = 0                      %                       m/s
        vz = 0                      %                       m/s
        ux = 0                      %Controls over time,    N
        uy = 0                      %                       N
        uz = 0                      %                       N
        
        %% Attitude
        wb1 = 0                     %Angular velocity       rad/s
        wb2 = 0                     %over time,             rad/s
        wb3 = 0                     %                       rad/s
        th1 = 0
        th2 = 0
        th3 = 0
        q1 = 0
        q2 = 0
        q3 = 0
        q4 = 1;

        point = 0
        pt = [0,0,0]
        
        %% Debug
        flag = []                   %Exit flag
        
    end
    properties (Dependent)
        m                           %Total mass,            kg
        mdot                        %Mass flow rate,        kg/s
        p                           %Position vector,       m
        v                           %Velocity vector,       m/s
        w                           %Angular velocity,
        qb                          %Quaternions
        R                           %Rotation matrix Rib
        ubnd                        %Upper bound,           m
        lbnd                        %Lower bound,           m
        I                           %Moments of Inertia,    kg/m^2
        
    end
    methods
        % Constructor
        function obj = satellite(umax,ISP,dryMass,fuel)
            if nargin > 0
                obj.umax = umax;
                obj.ISP = ISP;
                obj.dryMass = dryMass;
                obj.fuel = fuel;
            end
        end
        
        % Total mass (Dependent)
        function m = get.m(obj)
            m = obj.fuel+obj.dryMass;
        end
        % Mass flow rate (Dependent)
        function mdot = get.mdot(obj)
            mdot = 1/obj.ISP/9.81;
        end
        function p = get.p(obj)
            p = [obj.x(end),obj.y(end),obj.z(end)];
        end
        function v = get.v(obj)
            v = [obj.vx(end),obj.vy(end),obj.vz(end)];
        end
        function w = get.w(obj)
            w = [obj.wb1(end),obj.wb2(end),obj.wb3(end)]';
        end
        function qb = get.qb(obj)
            qb = [obj.q1(end),obj.q2(end),obj.q3(end),obj.q4(end)]';
        end
        function R = get.R(obj)
                q = obj.qb(1:3);
                q4 = obj.qb(4);
                qx = [0 -q(3) q(2)
                      q(3) 0 -q(1)
                     -q(2) q(1) 0];
                Rbi = (q4^2-q'*q)*eye(3)+2*(q*q')-2*q4*qx;
                R = Rbi';
        end
        function ubnd = get.ubnd(obj)
            ubnd = obj.bnd/2;
        end
        function lbnd = get.lbnd(obj)
            lbnd = -obj.bnd/2;
        end
        function I = get.I(obj)
            I(1) = 1/12*obj.m*(obj.bnd(2)^2+obj.bnd(3)^2);
            I(2) = 1/12*obj.m*(obj.bnd(1)^2+obj.bnd(3)^2);
            I(3) = 1/12*obj.m*(obj.bnd(1)^2+obj.bnd(2)^2);
        end      
        %% Basic point to point movement with collision avoidance
        function sat = approach(sat,scenario,p,lbnd,ubnd)
            if nargin < 4 || isempty(lbnd)
                lbnd = [];
                ubnd = [];
            end
            if sat.fuel > 0
                w1 = 1e-1; %Thrust
                w2 = 1;    %Targeting
                Nvar = scenario.Nvar;
                Neom = scenario.Neom;
                scenario.Nobj = size(lbnd,1);
                Nbi = scenario.Nbi;
                Ntotal = scenario.Ntotal;
                
                % Function coefficients
                f = [w1*scenario.dt*ones(Nvar,1); %Control thrusts
                    zeros(Neom,1);       %HCW accelerations
                    w2*ones(3,1);        %Target distance
                    zeros(Nbi,1)];       %Collision avoidance
                
                % Parameter bounds, lower & upper
                lb = [zeros(Nvar,1);   %Control thrusts
                    -inf*ones(Neom,1); %HCW accelerations
                    zeros(3,1);        %Target distance
                    zeros(Nbi,1)];     %Collision avoidance
                
                ub = [ones(Nvar,1);   %Control thrusts
                    inf*ones(Neom,1); %HCW accelerations
                    inf*ones(3,1);    %Target distance
                    ones(Nbi,1)];     %Collision avoidance
                
                % Integer constraints
                intcon = [1:Nvar,Nvar+Neom+3+1:Ntotal];
                
                % Equality contraints
                Aeq = []; beq = [];
                [Aeq,beq] = setEOM(Aeq,beq,sat,scenario);
                
                % Inequality contraints
                A = [];   b = [];
                [A,b] = minDistance(A,b,sat,scenario,p);
                [A,b] = maxVelocity(A,b,sat,scenario);
                
                for ii = 1:size(lbnd,1)
                    [A,b] = addObstacle(A,b,sat,scenario,lbnd(ii,:),ubnd(ii,:),ii);
                end

                options = optimoptions(@intlinprog,'Display','None','MaxTime',1);
                [u,~,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub,options);
                
                sat = signalsProp(sat,scenario,u,exitflag);
            else
                sat = driftProp(sat,scenario);
            end
        end
        
        %% Maximize distance from point with collision avoidance
%         function sat = evade(sat,scenario,p)
%         end
        
        %% Hold within a certain zone
        function sat = maintain(sat,scenario,lbnd,ubnd)
            if sat.fuel > 0
                Nvar = scenario.Nvar;
                Neom = scenario.Neom;
                
                % Function coefficients
                f = [scenario.dt*ones(Nvar,1);  %Control thrusts
                     zeros(Neom,1)              %HCW accelerations
                     zeros(3,1)];               %Target distance
                
                % Parameter bounds, lower & upper
                lb = [zeros(Nvar,1);            %Control thrusts
                    -inf*ones(Neom,1)           %HCW accelerations
                     zeros(3,1)];
                
                ub = [ones(Nvar,1);             %Control thrusts
                     inf*ones(Neom,1)           %HCW accelerations
                     ones(3,1)];    
                 
                % Integer constraints
                intcon = 1:Nvar;
                
                % Equality contraints
                Aeq = []; beq = [];
                [Aeq,beq] = setEOM(Aeq,beq,sat,scenario);
                
                % Inequality contraints
                A = [];   b = [];
                [A,b] = holdProximity(A,b,sat,scenario,lbnd,ubnd);
                [A,b] = maxVelocity(A,b,sat,scenario);
                
                options = optimoptions(@intlinprog,'Display','None','MaxTime',1);
                [u,~,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub,options);
                
                sat = signalsProp(sat,scenario,u,exitflag);
            else
                sat = driftProp(sat,scenario);
            end
        end
    end
end

function sat = signalsProp(sat,scenario,u,exitflag)
Nvar = scenario.Nvar;

iter = length(sat.x);
sat.flag(iter) = exitflag;

% Control signals
u = round(u);
sat.ux(iter) = sat.umax*(u(1)-u(2));
sat.uy(iter) = sat.umax*(u(3)-u(4));
sat.uz(iter) = sat.umax*(u(5)-u(6));
ax = u(Nvar+1);
ay = u(Nvar+2);
az = u(Nvar+3);

% New velocity
dt = scenario.dt;
sat.vx(iter+1) = sat.vx(iter)+(sat.ux(iter)/sat.m+ax)*dt;
sat.vy(iter+1) = sat.vy(iter)+(sat.uy(iter)/sat.m+ay)*dt;
sat.vz(iter+1) = sat.vz(iter)+(sat.uz(iter)/sat.m+az)*dt;

% New position
sat.x(iter+1) = sat.x(iter)+sat.vx(iter)*dt;
sat.y(iter+1) = sat.y(iter)+sat.vy(iter)*dt;
sat.z(iter+1) = sat.z(iter)+sat.vz(iter)*dt;

sat.ux(iter+1) = 0;
sat.uy(iter+1) = 0;
sat.uz(iter+1) = 0;

% Reaction wheel torques
if sat.point
%     vt = (sat.pt-sat.p)/norm(sat.pt-sat.p);
%     th3 = atan2(vt(2),vt(1));
%     th2 = atan2(vt(3),norm([vt(1),vt(2)]));
%     R = rot(th2,2)*rot(th3,3);
%     q4t = 1/2*sqrt(1+trace(R));
%     qt = 1/4/q4t*[R(2,3)-R(3,2);R(3,1)-R(1,3);R(1,2)-R(2,1)];
% 
%     Tx = -sat.kp*(sat.q1(iter)-qt(1))-sat.kd*sat.wb1(iter);
%     Ty = -sat.kp*(sat.q2(iter)-qt(2))-sat.kd*sat.wb2(iter);
%     Tz = -sat.kp*(sat.q3(iter)-qt(3))-sat.kd*sat.wb3(iter);   
else
    Tx = -sat.kp*(sat.q1(iter))-sat.kd*sat.wb1(iter);
    Ty = -sat.kp*(sat.q2(iter))-sat.kd*sat.wb2(iter);
    Tz = -sat.kp*(sat.q3(iter))-sat.kd*sat.wb3(iter);
end

if Tx > sat.Tmax,   Tx = sat.Tmax;  end
if Tx < -sat.Tmax,  Tx = -sat.Tmax; end
if Ty > sat.Tmax,   Ty = sat.Tmax;  end
if Ty < -sat.Tmax,  Ty = -sat.Tmax; end
if Tz > sat.Tmax,   Tz = sat.Tmax;  end
if Tz < -sat.Tmax,  Tz = -sat.Tmax; end

% Total applied torque
M(1) = sat.uz(iter)*sat.dy-sat.uy(iter)*sat.dz+Tx;
M(2) = sat.ux(iter)*sat.dz-sat.uz(iter)*sat.dx+Ty;
M(3) = sat.uy(iter)*sat.dx-sat.ux(iter)*sat.dy+Tz;

% New angular velocity
I = sat.I;
w = sat.w;
sat.wb1(iter+1) = sat.wb1(iter)+((I(2)-I(3))/I(1)*w(2)*w(3)+M(1)/I(1))*dt;
sat.wb2(iter+1) = sat.wb2(iter)+((I(3)-I(1))/I(2)*w(1)*w(3)+M(2)/I(2))*dt;
sat.wb3(iter+1) = sat.wb3(iter)+((I(1)-I(2))/I(3)*w(1)*w(2)+M(3)/I(3))*dt;

sat.th1(iter+1) = sat.th1(iter)+sat.wb1(iter)*dt;
sat.th2(iter+1) = sat.th2(iter)+sat.wb2(iter)*dt;
sat.th3(iter+1) = sat.th3(iter)+sat.wb3(iter)*dt;

% New quaternions
q = sat.qb(1:3);
q4 = sat.qb(4);
qx = [0   -q(3) q(2)
      q(3) 0   -q(1)
     -q(2) q(1) 0];
 
dqdt = 1/2*[qx+q4*eye(3);-q']*w;
sat.q1(iter+1) = sat.q1(iter)+dqdt(1)*dt;
sat.q2(iter+1) = sat.q2(iter)+dqdt(2)*dt;
sat.q3(iter+1) = sat.q3(iter)+dqdt(3)*dt;
sat.q4(iter+1) = sat.q4(iter)+dqdt(4)*dt;

% Fuel mass loss
sat.fuel = sat.fuel-sum(u(1:6))*sat.umax*sat.mdot;
end

% No fuel - drift
function sat = driftProp(sat,scenario)
dt = scenario.dt;
switch sat.EOM
    case 'HCW'
        A = HCW(scenario);
    case 'LERM'
        A = LERM(scenario);
end
iter = length(sat.x);
X = [sat.x(iter),sat.y(iter),sat.z(iter),...
    sat.vx(iter),sat.vy(iter),sat.vz(iter)]';
DX = A*X;

sat.ux(iter) = 0;
sat.uy(iter) = 0;
sat.uz(iter) = 0;
sat.vx(iter+1) = sat.vx(iter)+DX(4)*dt;
sat.vy(iter+1) = sat.vy(iter)+DX(5)*dt;
sat.vz(iter+1) = sat.vz(iter)+DX(6)*dt;
sat.x(iter+1) = sat.x(iter)+sat.vx(iter)*dt;
sat.y(iter+1) = sat.y(iter)+sat.vy(iter)*dt;
sat.z(iter+1) = sat.z(iter)+sat.vz(iter)*dt;
sat.ux(iter+1) = 0;
sat.uy(iter+1) = 0;
sat.uz(iter+1) = 0;
sat.flag(iter) = 3;
end