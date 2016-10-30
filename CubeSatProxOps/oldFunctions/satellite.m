classdef satellite
    %satellite defines the properties of a satellite as well as its trajectory
    % relative to the origin
    
    % Default properties of a basic cubesat
    properties
        name = ''
        EOM = 'HCW'
        mode = 'approach'
        color = 'b'
        umax = 0.25                 %N
        ISP = 150                   %s
        dryMass = 10                %kg
        fuel = 0.5                  %kg
        vmax = 0.5                  %m/s
        x = 0                       %m
        y = 0                       %m
        z = 0                       %m
        vx = 0                      %m/s
        vy = 0                      %m/s
        vz = 0                      %m/s
        ux = 0                      %N
        uy = 0                      %N
        uz = 0                      %N
        lbnd = [-0.05,-0.05,-0.05]  %m
        ubnd = [0.05,0.05,0.05]     %m
        flag = []
    end
    properties (Dependent)
        m                           %kg
        mdot                        %kg/s
        p                           %m
        v                           %m/s
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
                if isempty(lbnd) == 0
                    scenario.Nobj = 1;
                end
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
                
                if isempty(lbnd) == 0
                    [A,b] = addObstacle(A,b,sat,scenario,lbnd,ubnd,1);
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
                %[A,b] = holdProximity(A,b,sat,scenario,lbnd,ubnd);
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

sat.fuel = sat.fuel-sum(u(1:6))*sat.umax*sat.mdot;
end

% No fuel - drift
function sat = driftProp(sat,scenario)
dt = scenario.dt;
switch sat.EOM
    case 'HCW'
        A = HCW(scenario.n);
    case 'LERM'
        A = LERM(scenario.t,scenario.mu,scenario.a,scenario.ecc);
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