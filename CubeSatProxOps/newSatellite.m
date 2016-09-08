classdef newSatellite
    %satellite defines the properties of a satellite as well as its trajectory
    % relative to the origin
    
    % Default properties of a basic cubesat
    properties
        name = 'CubeSat'            %Satellite name
        EOM = 'HCW'                 %Relative motion model
        mode = 'approach'           %Satellite objective
        color = 'b'                 %Graph color
        
        umax = 0.25                 %Thrust,                N
        ISP = 150                   %Specific impulse,      s
        dryMass = 10                %Dry mass,              kg
        fuel = 0.5                  %Fuel mass,             kg
        vmax = 0.5                  %Maximum velocity,      m/s
        
        bnd = [0.1,0.1,0.1]         %Satellite size,        m
        Tmax = 0.25
        kp = 0.1                    %Position damping
        kd = 0.7                    %Velocity damping
        ki = 0.3                    %Integral damping
        dx = 0.01                   %x-axis moment arm,     m
        dy = 0.01                   %y-axis moment arm,     m
        dz = 0.01                   %z-axis moment arm,     m
        o1 = 0
        o2 = 0
        o3 = 0
        
        x = 0                       %x position over time,  m
        y = 0                       %y position over time,  m
        z = 0                       %z position over time,  m
        vx = 0                      %Velocity over time,    m/s
        vy = 0                      %                       m/s
        vz = 0                      %                       m/s
        ux = 0                      %Controls over time,    N
        uy = 0                      %                       N
        uz = 0                      %                       N
        wx = 0                      %Angular velocity       rad/s
        wy = 0                      %over time,             rad/s
        wz = 0                      %                       rad/s
        thx = 0                     %Euler angles           rad
        thy = 0                     %over time,             rad
        thz = 0                     %                       rad
        
        flag = []                   %Exit flag
    end
    properties (Dependent)
        m                           %Total mass,            kg
        mdot                        %Mass flow rate,        kg/s
        p                           %Position vector,       m
        v                           %Velocity vectory,      m/s
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
                Nhcw = scenario.Nhcw;
                scenario.Nobj = size(lbnd,1);
                Nbi = scenario.Nbi;
                Ntotal = scenario.Ntotal;
                
                % Function coefficients
                f = [w1*scenario.dt*ones(Nvar,1); %Control thrusts
                    zeros(Nhcw,1);       %HCW accelerations
                    w2*ones(3,1);        %Target distance
                    zeros(Nbi,1)];       %Collision avoidance
                
                % Parameter bounds, lower & upper
                lb = [zeros(Nvar,1);   %Control thrusts
                    -inf*ones(Nhcw,1); %HCW accelerations
                    zeros(3,1);        %Target distance
                    zeros(Nbi,1)];     %Collision avoidance
                
                ub = [ones(Nvar,1);   %Control thrusts
                    inf*ones(Nhcw,1); %HCW accelerations
                    inf*ones(3,1);    %Target distance
                    ones(Nbi,1)];     %Collision avoidance
                
                % Integer constraints
                intcon = [1:Nvar,Nvar+Nhcw+3+1:Ntotal];
                
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
        function sat = evade(sat,scenario,p)
        end
        
        %% Hold within a certain zone
        function sat = maintain(sat,scenario,lbnd,ubnd)
            if sat.fuel > 0
                Nvar = scenario.Nvar;
                Nhcw = scenario.Nhcw;
                
                % Function coefficients
                f = [scenario.dt*ones(Nvar,1);  %Control thrusts
                     zeros(Nhcw,1)              %HCW accelerations
                     zeros(3,1)];               %Target distance
                
                % Parameter bounds, lower & upper
                lb = [zeros(Nvar,1);            %Control thrusts
                    -inf*ones(Nhcw,1)           %HCW accelerations
                     zeros(3,1)];
                
                ub = [ones(Nvar,1);             %Control thrusts
                     inf*ones(Nhcw,1)           %HCW accelerations
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

sat.ux(iter) = sat.umax*(u(1)-u(2));
sat.uy(iter) = sat.umax*(u(3)-u(4));
sat.uz(iter) = sat.umax*(u(5)-u(6));
ax = u(Nvar+1);
ay = u(Nvar+2);
az = u(Nvar+3);

dt = scenario.dt;
sat.vx(iter+1) = sat.vx(iter)+(sat.ux(iter)/sat.m+ax)*dt;
sat.vy(iter+1) = sat.vy(iter)+(sat.uy(iter)/sat.m+ay)*dt;
sat.vz(iter+1) = sat.vz(iter)+(sat.uz(iter)/sat.m+az)*dt;

sat.x(iter+1) = sat.x(iter)+sat.vx(iter)*dt;
sat.y(iter+1) = sat.y(iter)+sat.vy(iter)*dt;
sat.z(iter+1) = sat.z(iter)+sat.vz(iter)*dt;

sat.ux(iter+1) = 0;
sat.uy(iter+1) = 0;
sat.uz(iter+1) = 0;

Tx = -sat.kp*(sat.thx(iter)-sat.o1)-sat.kd*sat.wx(iter);
Ty = -sat.kp*(sat.thy(iter)-sat.o2)-sat.kd*sat.wy(iter);
Tz = -sat.kp*(sat.thz(iter)-sat.o3)-sat.kd*sat.wz(iter);

if Tx > sat.Tmax,Tx = sat.Tmax;end
if Tx < -sat.Tmax,Tx = -sat.Tmax;end
if Ty > sat.Tmax,Ty = sat.Tmax;end
if Ty < -sat.Tmax,Ty = -sat.Tmax;end
if Tz > sat.Tmax,Tz = sat.Tmax;end
if Tz < -sat.Tmax,Tz = -sat.Tmax;end
    

sat.wx(iter+1) = sat.wx(iter)+((sat.uz(iter)*sat.dy-sat.uy(iter)*sat.dz+Tx)/sat.I(1))*dt;
sat.wy(iter+1) = sat.wy(iter)+((sat.ux(iter)*sat.dz-sat.uz(iter)*sat.dx+Ty)/sat.I(2))*dt;
sat.wz(iter+1) = sat.wz(iter)+((sat.uy(iter)*sat.dx-sat.ux(iter)*sat.dy+Tz)/sat.I(3))*dt;

sat.thx(iter+1) = sat.thx(iter)+sat.wx(iter)*dt;
sat.thy(iter+1) = sat.thy(iter)+sat.wy(iter)*dt;
sat.thz(iter+1) = sat.thz(iter)+sat.wz(iter)*dt;

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