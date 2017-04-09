%% New Satellite
classdef newSatellite < handle
    %% Default properties of a basic 6U cubesat
    properties
        %% Model
        name = 'CubeSat'            %Satellite name
        EOM = 'LERM'                %Relative motion model
        mode = 'approach'           %Satellite objective
        color = 'b'                 %Graph color
        
        %% Scenario
        scenario = newScenario
        
        %% Satellite parameters   
        dryMass = 6                 %Dry mass,              kg
        fuel = 0.5                  %Fuel mass,             kg
        bnd = [0.1,0.3,0.2]         %Satellite size,        m
        
        umax = 0.25                 %Thrust,                N
        canThrottle = 0;
        Isp = 150                   %Specific impulse,      s
        vmax = 0.5                  %Max velocity,          m/s
        
        Tmax = 0.0007               %Max reaction torque    Nm
        Lmax = 0.009351             %Max momentum storage,  Nms
        kp = 0.1                    %Position damping
        kd = 0.1                    %Velocity damping
        d = [0,0,0]                 %Thruster misalignment  m
        dv = 0                      %DeltaV                 m/s
        
        %% Trajectory
        t = 0                       %Time,                  s
        x = 0                       %x position over time,  m
        y = 0                       %y position over time,  m
        z = 0                       %z position over time,  m
        vx = 0                      %x velocity over time,  m/s
        vy = 0                      %y velocity over time,  m/s
        vz = 0                      %z velocity over time,  m/s
        ux = []                     %x thrust over time,    N
        uy = []                     %y thrust over time,    N
        uz = []                     %z thrust over time,    N
        ub1 = []                    %b1 thrust over time,   N
        ub2 = []                    %b2 thrust over time,   N
        ub3 = []                    %b3 thrust over time,   N
        T1 = []                     %x reaction torque,     Nm
        T2 = []                     %y reaction torque,     Nm
        T3 = []                     %z reaction torque,     Nm
        
        %% Attitude
        wb1 = 0                     %Angular velocity       rad/s
        wb2 = 0                     %over time              rad/s
        wb3 = 0                     %                       rad/s
        q1 = 0                      %Quaternions
        q2 = 0                      %over time
        q3 = 0                      %
        q4 = 1;                     %Quaternion scalar
        L1 = 0;                     %Stored Momentum,       Nms
        L2 = 0;
        L3 = 0;
        point = 0                   %Attitude pointing      binary
        pt = [0,0,0]                %Attitude point target  m
        
        %% MPC Solver
        T = 15                      %Horizon time,          s
        Nobj = 0
        Nslack = 0
        u = []                      %Current u output
        eom = []                    %Current state ouput
        J = 0                       %Cost fucntion over time
        flag = []                   %Exit flag
        
        %% Power
%         pPanel = []
%         pLoad = [] 
        
        %% STK
        app = [];
        root = [];
        
    end
    
    properties (Dependent)
        %% Mass
        m                           %Total mass,            kg
        mdot                        %Mass flow rate,        kg/s
        fuelUsed                    %Total fuel used,       kg

        %% Trajectory
        p                           %Position vector,       m
        v                           %Velocity vector,       m/s
        range
        
        %% Attitude
        w                           %Angular velocity,      rad/s
        qb                          %Quaternions vector,
        Rib                         %Rotation matrix b2i
        Rbi                         %Rotation matrix i2b
        ubnd                        %Upper bound,           m
        lbnd                        %Lower bound,           m
        I                           %Moments of Inertia     kg/m^2
        dv_ideal                    %Theoretical total dv   m/s
        dw_avail                    %Available dw,          rad/
        
        %% Solver
        Nsim
        Nvar
        Neom
        Nbi
        Ntotal
        dsafe
    end
    
    methods
        % Insert satellite in scenario
        function obj = newSatellite(scenario)
            if nargin > 0
                obj.scenario = scenario;
            end
        end
        
        % Number of simulation time steps
        function Nsim = get.Nsim(obj)
            Nsim = length(0:obj.scenario.dt:obj.T)-1;
        end
        
        % Number of control variables
        function Nvar = get.Nvar(obj)
            Nvar = 6*obj.Nsim;
        end
        
        % Number of hcw acceleration terms
        function Neom = get.Neom(obj)
            Neom = 3*obj.Nsim;
        end
        
        % Number of collision avoidance binary variables
        function Nbi = get.Nbi(obj)
            Nbi = obj.Nvar*obj.Nobj;
        end
        
        % Total number of variables
        function Ntotal = get.Ntotal(obj)
            Ntotal = obj.Nvar+obj.Neom+obj.Nslack+obj.Nbi;
        end
        
        % Total mass (Dependent)
        function dsafe = get.dsafe(obj)
            dsafe = max(obj.bnd)/2+sqrt(2)/2*obj.vmax*obj.scenario.dt;
        end
        
        % Total mass (Dependent)
        function m = get.m(obj)
            m = obj.fuel(end)+obj.dryMass;
        end
        
        % Mass flow rate (Dependent)
        function mdot = get.mdot(obj)
            mdot = obj.umax/obj.Isp/9.81;
        end
        
        % Fuel used
        function fuelUsed = get.fuelUsed(obj)
            fuelUsed = obj.fuel(1)-obj.fuel(end);
        end
        
        % Current position vector
        function p = get.p(obj)
            p = [obj.x(end),obj.y(end),obj.z(end)];
        end
        
        % Current velocity vector
        function v = get.v(obj)
            v = [obj.vx(end),obj.vy(end),obj.vz(end)];
        end
        
        function range = get.range(obj)
            range = sqrt(obj.x.^2+obj.y.^2+obj.z.^2);
        end
        
        % Current body-frame angular velocity
        function w = get.w(obj)
            w = [obj.wb1(end),obj.wb2(end),obj.wb3(end)]';
        end
        
        % Current quaternions
        function qb = get.qb(obj)
            qb = [obj.q1(end),obj.q2(end),obj.q3(end),obj.q4(end)]';
        end
        
        % Current intertial to body rotation matrix
        function Rbi = get.Rbi(obj)
            q = obj.qb(1:3);
            q4 = obj.qb(4);
            qx = [0   -q(3) q(2)
                q(3) 0   -q(1)
                -q(2) q(1) 0];
            Rbi = (q4^2-q'*q)*eye(3)+2*(q*q')-2*q4*qx;
            
        end
        
        % Current body to inertial rotation matrix
        function Rib = get.Rib(obj)
            Rib = transpose(obj.Rbi);
        end
        
        % Satellite body upper bound
        function ubnd = get.ubnd(obj)
            ubnd = obj.bnd/2;
        end
        
        % Satellite body lower lound
        function lbnd = get.lbnd(obj)
            lbnd = -obj.bnd/2;
        end
        
        % Satellite moment of inertia vector
        function I = get.I(obj)
            I(1) = 1/12*obj.m*(obj.bnd(2)^2+obj.bnd(3)^2);
            I(2) = 1/12*obj.m*(obj.bnd(1)^2+obj.bnd(3)^2);
            I(3) = 1/12*obj.m*(obj.bnd(1)^2+obj.bnd(2)^2);
        end
        
        % Ideal total dv
        function dv_ideal = get.dv_ideal(obj)
            dv_ideal = obj.Isp*9.81*...
                log((obj.dryMass+obj.fuel(1))/obj.dryMass);
        end
        
        % Available change in angular velocity
        function dw_avail = get.dw_avail(obj)
           L = [obj.L1(end),obj.L2(end),obj.L3(end)];
           L_avail = obj.Lmax-abs(L);
           dw_avail = (L_avail./obj.I);
        end
        
        %% Basic point to point movement with collision avoidance
        function sat = approach(sat,p,lbnd,ubnd)
            if nargin < 4 || isempty(lbnd)
                lbnd = [];ubnd = [];
            end
            if sat.fuel > 0
                w1 = 1e-1;                      %Thrust weight
                w2 = 1;                         %Targeting weight
                sat.Nslack = 3;
                sat.Nobj = size(lbnd,1);
                
                % Function coefficients
                dt = sat.scenario.dt;
                f = [w1*dt*ones(sat.Nvar,1);    %Control thrusts
                    zeros(sat.Neom,1);          %EOM accelerations
                    w2*ones(sat.Nslack,1);      %Target distance
                    zeros(sat.Nbi,1)];          %Collision avoidance
                
                % Parameter bounds, lower & upper
                lb = [zeros(sat.Nvar,1);        %Control thrusts
                    -inf*ones(sat.Neom,1);      %EOM accelerations
                    zeros(sat.Nslack,1);        %Target distance
                    zeros(sat.Nbi,1)];          %Collision avoidance
                ub = [ones(sat.Nvar,1);         %Control thrusts
                    inf*ones(sat.Neom,1);       %EOM accelerations
                    inf*ones(sat.Nslack,1);     %Target distance
                    ones(sat.Nbi,1)];           %Collision avoidance
                
                % Integer constraints
                if sat.canThrottle
                    intcon = [];
                else
                    intcon = [1:sat.Nvar,sat.Nvar+sat.Neom+3+1:sat.Ntotal];
                end
                
                % Equality contraints
                Aeq = []; beq = [];
                [Aeq,beq] = setEOM(Aeq,beq,sat);
                
                % Inequality contraints
                A = [];   b = [];
                [A,b] = minDistance(A,b,sat,p);
                [A,b] = maxVelocity(A,b,sat);
                
                for ii = 1:size(lbnd,1)
                    [A,b] = addObstacle(A,b,sat,lbnd(ii,:),ubnd(ii,:),ii);
                end
                
                iter = length(sat.x);
                options = optimoptions(@intlinprog,'Display','None','MaxTime',dt);
                [U,fval,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub,options);
                
                if isempty(U) == 1
                    sat.u = sat.u(7:end);
                    sat.eom = sat.eom(4:end);
                    sat.J(iter) = sat.J(iter-1);
                    sat.flag(iter) = 3;
                else
                    sat.u = U(1:sat.Nvar);
                    sat.eom = U(sat.Nvar+1:sat.Nvar+sat.Neom);
                    sat.J(iter) = fval;
                    sat.flag(iter) = exitflag;
                end
                sat.propSignals;
            else
                sat.driftProp;
            end
            sat.propAttitude;
            sat.t(iter+1) = sat.t(iter)+sat.scenario.dt;
        end
        
        %% Hold within a certain zone
        function sat = maintain(sat,lbnd,ubnd)
            if sat.fuel > 0
                sat.Nslack = 0;
                sat.Nobj = 0;
                dt = sat.scenario.dt;
                
                % Function coefficients
                f = [ones(sat.Nvar,1);           %Control thrusts
                     zeros(sat.Neom,1)];            %HCW accelerations
                
                % Parameter bounds, lower & upper
                lb = [zeros(sat.Nvar,1)             %Control thrusts
                    -inf*ones(sat.Neom,1)];        %HCW accelerations
                ub = [ones(sat.Nvar,1);             %Control thrusts
                    inf*ones(sat.Neom,1)];        %HCW accelerations
                
                % Integer constraints
                if sat.canThrottle
                    intcon = [];         
                else
                    intcon = 1:sat.Nvar;
                end
                
                % Equality contraints
                Aeq = []; beq = [];
                [Aeq,beq] = setEOM(Aeq,beq,sat);
                
                % Inequality contraints
                A = [];   b = [];
                [A,b] = holdProximity(A,b,sat,lbnd,ubnd);
                [A,b] = maxVelocity(A,b,sat);
                
                options = optimoptions(@intlinprog,'Display','None','MaxTime',dt);
                [U,fval,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub,options);%
                
                iter = length(sat.x);
                if isempty(U) == 1
                    sat.u = sat.u(7:end);
                    sat.eom = sat.eom(4:end);
                    sat.J(iter) = sat.J(iter-1);
                    sat.flag(iter) = 3;
                else
                    sat.u = U(1:sat.Nvar);
                    sat.eom = U(sat.Nvar+1:end);
                    sat.J(iter) = fval;
                    sat.flag(iter) = exitflag;
                end
                sat.propSignals;
            else
                sat.driftProp;
            end
            sat.t(iter+1) = sat.t(iter)+sat.scenario.dt;
            sat.propAttitude;
        end
        
        %% MPC Long-Range Maneuver
        function sat = phaseManeuver(sat,Xf,tf,dtM)
            T0 = sat.scenario.T;
            dt0 = sat.scenario.dt;
            sat.Nslack = 6;
            
            for nn = 1:(tf/dtM)
                sat.T = (tf-nn);
                sat.scenario.dt = dtM;
                dt = sat.scenario.dt;
                Nvar = sat.Nvar;
                
                % Optimization weights
                w1 = 1e-2;          %Control weight
                w2 = 1;             %Postion weight
                w3 = 1e3;           %Velocity weight
                
                % Function coefficients
                f = [w1*dt*ones(Nvar,1) %Control thrusts
                    zeros(sat.Neom,1)
                    w2*ones(3,1)
                    w3*ones(3,1)];
                
                % Parameter bounds, lower & upper
                lb = [zeros(sNvar,1);   %Control thrusts
                    -inf*ones(sat.Neom,1);
                    zeros(6,1)];
                
                ub = [ones(Nvar,1);   %Control thrusts
                    inf*ones(sat.Neom,1);
                    1e5*ones(6,1)];
                
                % Equality contraints
                Aeq = []; beq = [];
                [Aeq,beq] = setEOM(Aeq,beq,sat,eye(3));
                
                % Inequality contraints
                A = [];   b = [];
                [A,b] = setPhaseState(A,b,sat,Xf);
                
                % Integer constraint
                intcon = [];
                
                % options = optimoptions(@linprog);
                [U,fval,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub);
                
                sat.scenario.dt = dt0;
                if isempty(U)
                    sat.propagate(dtM/dt0);
                else
                    for ii = 1:(dtM/dt0)
                        iter = length(sat.x);
                        sat.J(iter) = fval;
                        sat.flag(iter) = exitflag;
                        sat.u(1:2:6) = sat.Rbi*U(1:2:6);
                        sat.u(2:2:6) = sat.Rbi*U(2:2:6);
                        sat.eom = U(Nvar+1:Nvar+3);
                        sat.propSignals;
                        sat.propAttitude;
                        sat.t(iter+1) = sat.t(iter)+sat.scenario.dt;
                    end
                end
            end
            sat.T = T0;
        end
        
        %% Equality contrainted LRM
        function sat = phaseManeuverEq(sat,Xf,tf,dtM)
            % Save old parameters
            T0 = sat.T;
            dt0 = sat.scenario.dt;
            umax0 = sat.umax;
            
            % Setup solver
            sat.T = tf;
            sat.scenario.dt = dtM;
            sat.umax = sat.umax/sqrt(2);
            
            % No slack, no obstacles
            sat.Nslack = 0;
            sat.Nobj = 0;
            Nvar = sat.Nvar;
            Neom = sat.Neom;
            
            % Function coefficients
            f = [dtM*ones(Nvar,1)
                 zeros(Neom,1)];
            
            % Parameter bounds, lower & upper
            lb = [zeros(Nvar,1) 
                 -inf*ones(Neom,1)];
            ub = [ones(Nvar,1)
                  inf*ones(Neom,1)];
            
            % Equality contraints
            Aeq = []; beq = [];
            [Aeq,beq] = setEOM(Aeq,beq,sat,eye(3));
            [Aeq,beq] = setPhaseStateEq(Aeq,beq,sat,Xf);
            
            % Inequality contraints
            A = [];   b = [];
            
            % Integer constraint
            intcon = [];
            
            % Optimizer
            options = optimoptions(@intlinprog,'Display','None');
            [U,fval,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub,options);
            
            sat.scenario.dt = dt0;
            jj = 1;
            for ii = 1:6:(Nvar)
                for kk = 1:(dtM/dt0)
                    iter = length(sat.x);
                    sat.J(iter) = fval;
                    sat.flag(iter) = exitflag;
                    
                    sat.u(1:2:6) = sat.Rbi*U(ii:2:ii+5);
                    sat.u(2:2:6) = sat.Rbi*U(ii+1:2:ii+5);
                    sat.eom = U(Nvar+jj:Nvar+jj+2);
                    
                    sat.propSignals;
                    sat.propAttitude;
                    sat.t(iter+1) = sat.t(iter)+dt0;
                end
                jj = jj+3;
            end
            sat.T = T0;
            sat.umax = umax0;
        end
        
        %% Propagate with no control for given time
        function sat = propagate(sat,tspan)
            for ii = 1:sat.scenario.dt:tspan
                iter = length(sat.x);
                sat.J(iter) = 0;
                sat.flag(iter) = 3;
                sat.driftProp;
                sat.propAttitude;
                sat.t(iter+1) = sat.t(iter)+sat.scenario.dt;
            end
        end
        
        %% Convert solver signals to propagation
        function sat = propSignals(sat)
            U = sat.u;
            iter = length(sat.x);
            
            % Control signals
            ub = ([U(1)-U(2)
                   U(3)-U(4)
                   U(5)-U(6)]);
            ui = sat.Rib*ub;
            
            sat.ub1(iter) = sat.umax*ub(1);
            sat.ub2(iter) = sat.umax*ub(2);
            sat.ub3(iter) = sat.umax*ub(3);
            sat.ux(iter) = sat.umax*ui(1);
            sat.uy(iter) = sat.umax*ui(2);
            sat.uz(iter) = sat.umax*ui(3);
            
            ax = sat.eom(1);
            ay = sat.eom(2);
            az = sat.eom(3);
            
            dt = sat.scenario.dt;
                          
            % New velocity
            sat.vx(iter+1) = sat.vx(iter)+(sat.ux(iter)/sat.m+ax)*dt;
            sat.vy(iter+1) = sat.vy(iter)+(sat.uy(iter)/sat.m+ay)*dt;
            sat.vz(iter+1) = sat.vz(iter)+(sat.uz(iter)/sat.m+az)*dt;
            
            % New position
            sat.x(iter+1) = sat.x(iter)+sat.vx(iter)*dt;
            sat.y(iter+1) = sat.y(iter)+sat.vy(iter)*dt;
            sat.z(iter+1) = sat.z(iter)+sat.vz(iter)*dt;
            
            % Resize control vector
            sat.J(iter+1) = sat.J(iter);
            sat.ux(iter+1) = 0;
            sat.uy(iter+1) = 0;
            sat.uz(iter+1) = 0;
            sat.ub1(iter+1) = 0;
            sat.ub2(iter+1) = 0;
            sat.ub3(iter+1) = 0;
            
            % Delta v and fuel mass loss
            sat.dv(iter+1) = sat.dv(iter)+sum(abs(U(1:6)))*(sat.umax/sat.m)*dt;
            sat.fuel(iter+1) = sat.fuel(iter)-sum(abs(U(1:6)))*sat.mdot*dt;
        end
        
        %% Propagate trajectory with no fuel
        function sat = driftProp(sat)
            dt = sat.scenario.dt;
            
            % Runge kutta
            X = [sat.p';sat.v'];
            uzero = zeros(6,1);
            Xnew = RKDP(@trajectoryODE,sat.t(end),dt,X,sat,uzero);
            
            % Update state
            iter = length(sat.x);
            sat.x(iter+1) = Xnew(1);
            sat.y(iter+1) = Xnew(2);
            sat.z(iter+1) = Xnew(3);
            sat.vx(iter+1) = Xnew(4);
            sat.vy(iter+1) = Xnew(5);
            sat.vz(iter+1) = Xnew(6);
           
            % Update control vector
            sat.J(iter+1) = sat.J(iter);
            sat.ux(iter+1) = 0;
            sat.uy(iter+1) = 0;
            sat.uz(iter+1) = 0;
            sat.ub1(iter+1) = 0;
            sat.ub2(iter+1) = 0;
            sat.ub3(iter+1) = 0;
            
            % Fuel
            sat.dv(iter+1) = sat.dv(iter);
            sat.fuel(iter+1) = sat.fuel(iter);
        end
        
        %% Propagate attitude
        function sat = propAttitude(sat)
            % Reaction wheel torques
            iter = length(sat.q1);
            dt = sat.scenario.dt;
            
            % Uses PD contorolled reaction wheel torques to point
            % satellite at user-defined point.
            % Else the satellite just minimizes divergence from the
            % chief RIC axes.
            kp = sat.kp/sat.scenario.dt;
            kd = sat.kd/sat.scenario.dt;
            if sat.point == 1
                % Targeted pointing
                vt = (sat.pt-sat.p)/norm(sat.pt-sat.p);
                th3 = -atan2(vt(2),vt(1));
                th2 = -atan2(vt(3),norm([vt(1),vt(2)]));
                R = rot(th3,3)*rot(th2,2);
                                
                qr = zeros(4,1);
                qr(4) = 1/2*sqrt(1+trace(R))+1e-3;
                qr(1:3) = 1/(4*qr(4))*[R(2,3)-R(3,2);R(3,1)-R(1,3);R(1,2)-R(2,1)];
                qA = [qr(4)  qr(3) -qr(2) -qr(1)
                     -qr(3)  qr(4)  qr(1)  qr(2)
                      qr(2) -qr(1)  qr(4) -qr(3)
                      qr(1)  qr(2)  qr(3)  qr(4)];
                qe = qA*sat.qb;
                
                sat.T1(iter) = -kp*(qe(1)*qe(4))-kd*sat.wb1(iter);
                sat.T2(iter) = -kp*(qe(2)*qe(4))-kd*sat.wb2(iter);
                sat.T3(iter) = -kp*(qe(3)*qe(4))-kd*sat.wb3(iter);
                sat.T1(iter+1) = 0;
                sat.T2(iter+1) = 0;
                sat.T3(iter+1) = 0;
            elseif sat.point == 0
                % Stabilization
                sat.T1(iter) = -kp*(sat.q1(iter))-kd*sat.wb1(iter);
                sat.T2(iter) = -kp*(sat.q2(iter))-kd*sat.wb2(iter);
                sat.T3(iter) = -kp*(sat.q3(iter))-kd*sat.wb3(iter);
                sat.T1(iter+1) = 0;
                sat.T2(iter+1) = 0;
                sat.T3(iter+1) = 0;
            else
                % No torque
                sat.T1(iter) = 0;
                sat.T2(iter) = 0;
                sat.T3(iter) = 0;
                sat.T1(iter+1) = 0;
                sat.T2(iter+1) = 0;
                sat.T3(iter+1) = 0;
            end
            
            % Ensure torques are below max capable torque
            if sat.T1(iter) > sat.Tmax,   sat.T1(iter) = sat.Tmax;  end
            if sat.T1(iter) < -sat.Tmax,  sat.T1(iter) = -sat.Tmax; end
            if sat.T2(iter) > sat.Tmax,   sat.T2(iter) = sat.Tmax;  end
            if sat.T2(iter) < -sat.Tmax,  sat.T2(iter) = -sat.Tmax; end
            if sat.T3(iter) > sat.Tmax,   sat.T3(iter) = sat.Tmax;  end
            if sat.T3(iter) < -sat.Tmax,  sat.T3(iter) = -sat.Tmax; end
            
            % Total applied torque with thruster offset moment
            M(1) = sat.ub3(iter)*sat.d(2)-sat.ub2(iter)*sat.d(3)+sat.T1(iter);
            M(2) = sat.ub1(iter)*sat.d(3)-sat.ub3(iter)*sat.d(1)+sat.T2(iter);
            M(3) = sat.ub2(iter)*sat.d(1)-sat.ub1(iter)*sat.d(2)+sat.T3(iter);
            
            % New stored momentum
            I = sat.I;
            w = sat.w;
%             sat.L1(iter+1) = sat.L1(iter)-sat.T1(iter)*dt;
%             sat.L2(iter+1) = sat.L2(iter)-sat.T2(iter)*dt;
%             sat.L3(iter+1) = sat.L3(iter)-sat.T3(iter)*dt;
            sat.L1(iter+1) = sat.L1(iter)+I(1)*((I(2)-I(3))/I(1)*w(2)*w(3)+M(1)/I(1))*dt;
            sat.L2(iter+1) = sat.L2(iter)+I(2)*((I(3)-I(1))/I(2)*w(1)*w(3)+M(2)/I(2))*dt;
            sat.L3(iter+1) = sat.L3(iter)+I(3)*((I(1)-I(2))/I(3)*w(1)*w(2)+M(3)/I(3))*dt;
%   
            % Saturation check
%             if (sat.L1(iter+1) > sat.Lmax) || (sat.L1(iter+1) < -sat.Lmax)
%                 sat.T1(iter) = 0;
%                 sat.L1(iter+1) = sat.L1(iter);
%             end
%             if (sat.L2(iter+1) > sat.Lmax) || (sat.L2(iter+1) < -sat.Lmax)
%                 sat.T2(iter) = 0;
%                 sat.L2(iter+1) = sat.L2(iter);
%             end
%             if (sat.L3(iter+1) > sat.Lmax) || (sat.L3(iter+1) < -sat.Lmax)
%                 sat.T3(iter) = 0;
%                 sat.L3(iter+1) = sat.L3(iter);
%             end
            
            % Total applied torque with thruster offset moment
            M(1) = sat.ub3(iter)*sat.d(2)-sat.ub2(iter)*sat.d(3)+sat.T1(iter);
            M(2) = sat.ub1(iter)*sat.d(3)-sat.ub3(iter)*sat.d(1)+sat.T2(iter);
            M(3) = sat.ub2(iter)*sat.d(1)-sat.ub1(iter)*sat.d(2)+sat.T3(iter);
            
            % State vector
            X0 = [sat.wb1(iter)
                sat.wb2(iter)
                sat.wb3(iter)
                sat.q1(iter)
                sat.q2(iter)
                sat.q3(iter)
                sat.q4(iter)];
            
            % Runge-Kutta
            X = RKDP(@attitudeODE,sat.t(iter),dt,X0,sat,M);      
            sat.wb1(iter+1) = X(1);
            sat.wb2(iter+1) = X(2);
            sat.wb3(iter+1) = X(3);
            
            % Normalize quaternions
            X(4:7) = X(4:7)/norm(X(4:7));
            sat.q1(iter+1) = X(4);
            sat.q2(iter+1) = X(5);
            sat.q3(iter+1) = X(6);
            sat.q4(iter+1) = X(7);
        end
        
        %% Fix Attitude
        function fixAttitude(sat,qN)
           sat.wb1(end) = 0;
           sat.wb2(end) = 0;
           sat.wb3(end) = 0;
           sat.q1(end) = qN(1);
           sat.q2(end) = qN(2);
           sat.q3(end) = qN(3);
           sat.q4(end) = qN(4);
        end
        
        %% Fix Position
        function fixPosition(sat,pN)
            sat.x(end) = pN(1);
            sat.y(end) = pN(2);
            sat.z(end) = pN(3);
            sat.vx(end) = 0;
            sat.vy(end) = 0;
            sat.vz(end) = 0;
        end
        
        %% Plot relative trajectory
        function plotTrajectory(sat,lbnd,ubnd,unit,plume)
            if nargin < 2 || isempty(lbnd),lbnd = [];end
            if nargin < 3 || isempty(ubnd),ubnd = [];end
            if nargin < 4 || isempty(unit),unit = 5;end
            if nargin < 5 || isempty(plume),plume = 0;end
            c1 = [0,0.4470,0.7410];
            
            figure('Position',[100 100 1280 720]);
            hold on  
%             % Plot obstacle bounds
%             for ii = 1:size(lbnd,1)
%                 plotObstacle(lbnd(ii,:),ubnd(ii,:),'-k');
%             end
%             bnds = plot3(0,0,0,'-k');
            bnds = plot3(0,0,0,'ko','markerfacecolor','b','markersize',6);
            
            % Plot satellite trajectory     
            for jj = 1:length(sat)                
                p0 = plot3(sat.x(1),sat.y(1),sat.z(1),'ko','markerfacecolor','g','markersize',6);
                pf = plot3(sat.x(end),sat.y(end),sat.z(end),'ko','markerfacecolor','r','markersize',6);
                trj = plot3(sat.x,sat.y,sat.z,'-','linewidth',1,'color',c1);
                
                if plume
                    pl = quiver3(sat.x,sat.y,sat.z,-sat.ux,-sat.uy,-sat.uz,plume,'r','linewidth',1.5);
                end
                
                % Plot satellite body axes
                R = unit*sat.Rib;
                plot3([sat.p(1),sat.p(1)+R(1,1)'],[sat.p(2),sat.p(2)+R(2,1)'],...
                    [sat.p(3),sat.p(3)+R(3,1)'],'b','linewidth',1);
                plot3([sat.p(1),sat.p(1)+R(1,2)'],[sat.p(2),sat.p(2)+R(2,2)'],...
                    [sat.p(3),sat.p(3)+R(3,2)'],'r','linewidth',1);
                plot3([sat.p(1),sat.p(1)+R(1,3)'],[sat.p(2),sat.p(2)+R(2,3)'],...
                    [sat.p(3),sat.p(3)+R(3,3)'],'g','linewidth',1);
            end
            hold off
            
            % Axis labels
            grid on
            xl = xlabel('Radial, m');
            yl = ylabel('In-track, m');
            zl = zlabel('Cross-track, m');            
            if plume
                lg = legend([bnds,p0,pf,trj,pl],{'RSO','Initial Position',...
                    'Final Position','Relative Trajectory','Thrust Plume'},...
                    'location','best','orientation','horizontal');
            else
                lg = legend([bnds,p0,pf,trj],{'RSO','Initial Position','Final Position',...
                    'Relative Trajectory'},'location','best','orientation','horizontal');
            end
                     
            set([xl,yl,zl,lg],'FontSize',12)
            axis('equal','tight')
            view([90,90])
            camva(6)
            drawnow
            
        end
        
        %% Plot relative trajectory in multiple frames
        function subplotTrajectory(sat)
            
            figure('Position',[100 100 1280 720]);
         
            % 3D view
            axes('Position',[0.05 0.1 0.5 0.8]);
            axis('equal')
            camva(9)
            hold on
            plot3(sat.x,sat.y,sat.z,'linewidth',1);
            quiver3(sat.x,sat.y,sat.z,-sat.ux,-sat.uy,-sat.uz,10,'r','linewidth',1.5);
            p0 = plot3(sat.x(1),sat.y(1),sat.z(1),'ko','markerfacecolor','b','markersize',6);
            origin = plot3(0,0,0,'ko','markerfacecolor','r','markersize',6);
            hold off
            grid on
            %title('Relative Trajectory')
            xlabel('Radial, x, m')
            ylabel('In-Track, y, m')
            zlabel('Cross-Track, z, m')
            view(135,30)
            legend([origin,p0],{'RSO','Initial Position'},'location','northeast')
            
            % Plane 1
            axes('Position',[0.65 0.7 0.25 0.2]);
            hold on
            plot3(0,0,0,'ko','markerfacecolor','r','markersize',6);
            plot3(sat.x,sat.y,sat.z,'linewidth',1);
            quiver3(sat.x,sat.y,sat.z,-sat.ux,-sat.uy,-sat.uz,1,'r','linewidth',1.5);
            plot3(sat.x(1),sat.y(1),sat.z(1),'ko','markerfacecolor','b','markersize',6);        
            hold off
            grid on
            title('RI Plane')
            xlabel('Radial, x, m')
            ylabel('In-Track, y, m')
            zlabel('Cross-Track, z, m')
            view(0,90)
            
            % Plane 2
            axes('Position',[0.65 0.4 0.25 0.2]);
            hold on
            plot3(0,0,0,'ko','markerfacecolor','r','markersize',6);
            plot3(sat.x,sat.y,sat.z,'linewidth',1);
            quiver3(sat.x,sat.y,sat.z,-sat.ux,-sat.uy,-sat.uz,1,'r','linewidth',1.5);
            plot3(sat.x(1),sat.y(1),sat.z(1),'ko','markerfacecolor','b','markersize',6);
            hold off
            grid on
            title('RC Plane')
            xlabel('Radial, x, m')
            ylabel('In-Track, y, m')
            zlabel('Cross-Track, z, m')
            view(0,0)
            
            % Plane 3
            axes('Position',[0.65 0.1 0.25 0.2]);
            hold on
            plot3(0,0,0,'ko','markerfacecolor','r','markersize',6);
            plot3(sat.x,sat.y,sat.z,'linewidth',1);
            quiver3(sat.x,sat.y,sat.z,-sat.ux,-sat.uy,-sat.uz,1,'r','linewidth',1.5);
            plot3(sat.x(1),sat.y(1),sat.z(1),'ko','markerfacecolor','b','markersize',6);
            hold off
            grid on
            title('IC Plane')
            xlabel('Radial, x, m')
            ylabel('In-Track, y, m')
            zlabel('Cross-Track, z, m')
            view(90,0)
            
            text = annotation('textbox');
            text.FontSize = 9;
            text.String = sprintf('Fuel usage: %.2f g\nDelta V: %.2f m/s',...
                                  (sat.fuel(1)-sat.fuel(end))*1e3,sat.dv(end));
            text.BackgroundColor = [1,1,1];
            text.Position = [0.1 0.81 0.15 0.07];
            text.VerticalAlignment = 'middle';
            text.HorizontalAlignment = 'center';
            drawnow
        end
          
        %% Plot controls, velocity, position
        function plotState(sat)
            tf = sat.t(end);
            
            c1 = [0,0.4470,0.7410];
            c2 = [0.8500,0.3250,0.0980];
            c3 = [0.9290,0.6940,0.1250];
            
            % main view
            figure
            subplot(3,3,1)
            hold on
            stairs(sat.t,sat.ub1/sat.umax,'color',c1,'linewidth',1.5)
            plot([0 tf],[0 0],'--k','linewidth',1)
            axis([0 tf -1.5 1.5])
            grid on
            title('Control Signals vs Time')
            ylabel('ub1')
            
            subplot(3,3,4)
            hold on
            stairs(sat.t,sat.ub2/sat.umax,'color',c2,'linewidth',1.5)
            plot([0 tf],[0 0],'--k','linewidth',1)
            axis([0 tf -1.5 1.5])
            grid on
            ylabel('ub2')
            
            subplot(3,3,7)
            hold on
            stairs(sat.t,sat.ub3/sat.umax,'color',c3,'linewidth',1.5)
            plot([0 tf],[0 0],'--k','linewidth',1)
            axis([0 tf -1.5 1.5])
            grid on
            xlabel('Time [s]')
            ylabel('ub3')
            
            % Velocity
            subplot(3,3,2)
            plot(sat.t,sat.vx,'color',c1,'linewidth',1.5)
            axis([0 tf 0 1],'auto y')
            grid on
            ylabel('Vx [m/s]')
            title('Velocity vs Time')
            
            subplot(3,3,5)
            plot(sat.t,sat.vy,'color',c2,'linewidth',1.5)
            axis([0 tf 0 1],'auto y')
            grid on
            ylabel('Vy [m/s]')
            
            subplot(3,3,8)
            plot(sat.t,sat.vz,'color',c3,'linewidth',1.5)
            axis([0 tf 0 1],'auto y')
            grid on
            xlabel('Time [s]')
            ylabel('Vz [m/s]')
            
            % Position
            subplot(3,3,3)
            plot(sat.t,sat.x,'color',c1,'linewidth',1.5)
            axis([0 tf 0 1],'auto y')
            grid on
            ylabel('x [m]')
            title('Position vs Time')
            
            subplot(3,3,6)
            plot(sat.t,sat.y,'color',c2,'linewidth',1.5)
            axis([0 tf 0 1],'auto y')
            grid on
            ylabel('y [m]')
            
            subplot(3,3,9)
            plot(sat.t,sat.z,'color',c3,'linewidth',1.5)
            axis([0 tf 0 1],'auto y')
            grid on
            xlabel('Time [s]')
            ylabel('z [m]')
            drawnow
        end
        
        %% Plot angular velocity, quaternions, momentum
        function plotAttitude(sat)
            tf = sat.t(end);
            c1 = [0,0.4470,0.7410];
            c2 = [0.8500,0.3250,0.0980];
            c3 = [0.9290,0.6940,0.1250];
            TP = sat.scenario.TP;
            
            figure('Position',[100,100,1280,720]);
            subplot(3,1,1)
            hold on
            plot(sat.t/TP,sat.T1,'color',c1,'linewidth',1.5)
            plot(sat.t/TP,sat.T2,'color',c2,'linewidth',1.5)
            plot(sat.t/TP,sat.T3,'color',c3,'linewidth',1.5)
            hold off
            axis([0 tf/TP 0 1],'auto y')
            grid on
            legend({'T1','T2','T3'},'location','eastoutside')
            xl = xlabel('Time, Orbits');
            yl = ylabel('Nm');
            title('Reaction Torque')
            set([xl,yl],'FontSize',12)
            
            subplot(3,1,2)
            hold on
            plot(sat.t/TP,sat.wb1*(180/pi),'color',c1,'linewidth',1.5)
            plot(sat.t/TP,sat.wb2*(180/pi),'color',c2,'linewidth',1.5)
            plot(sat.t/TP,sat.wb3*(180/pi),'color',c3,'linewidth',1.5)
            hold off
            axis([0 tf/TP 0 1],'auto y')
            grid on
            legend({'\omega1','\omega2','\omega3'},'location','eastoutside')
            xl = xlabel('Time, Orbits');
            yl = ylabel('deg/s');
            title('Angular Velocity')
            set([xl,yl],'FontSize',12)
            
            subplot(3,1,3)
            hold on
            plot(sat.t/TP,sat.q1,'color',c1,'linewidth',1.5)
            plot(sat.t/TP,sat.q2,'color',c2,'linewidth',1.5)
            plot(sat.t/TP,sat.q3,'color',c3,'linewidth',1.5)
            plot(sat.t/TP,sat.q4,'-k','linewidth',1.5)
            hold off
            axis([0 tf/TP 0 1],'auto y')
            grid on
            xl = xlabel('Time, Orbits');
            legend({'q1','q2','q3','q4'},'location','eastoutside')
            title('Attitude Quaternions')
            set([xl,yl],'FontSize',12)
            
%             subplot(4,1,4)
%             hold on
%             plot(sat.t,sat.L1,'Linewidth',1.5)
%             plot(sat.t,sat.L2,'Linewidth',1.5)
%             plot(sat.t,sat.L3,'Linewidth',1.5)
% %             plot([0 tf],[sat.Lmax sat.Lmax],'--k','linewidth',1)
% %             plot([0 tf],-[sat.Lmax sat.Lmax],'--k','linewidth',1)
%             hold off
%             axis([0 tf 0 1],'auto y')
%             grid on
%             xlabel('Time, s')
%             ylabel('Stored Angular Momentum, Nms')
%             legend({'L1','L2','L3'},'location','eastoutside')
%             title('Stored Angular Momentum vs Time')
            
            drawnow
        end
        
        %% Plot control signals and state over time
        function plotControls(sat)
            tf = sat.t(end);
            
            % Signals and cost function
            figure
            subplot(4,1,1)
            hold on
            stairs(sat.t,sat.ub1/sat.umax,'-k','linewidth',1)
            plot([0 tf],[0 0],'--k','linewidth',1)
            axis([0 tf -1.5 1.5])
            grid on
            title('Control Signals vs Time')
            xlabel('Time [s]')
            ylabel('u_{b1}')
            
            subplot(4,1,2)
            hold on
            stairs(sat.t,sat.ub2/sat.umax,'-k','linewidth',1)
            plot([0 tf],[0 0],'--k','linewidth',1)
            axis([0 tf -1.5 1.5])
            grid on
            xlabel('Time [s]')
            ylabel('u_{b2}')
            
            subplot(4,1,3)
            hold on
            stairs(sat.t,sat.ub3/sat.umax,'-k','linewidth',1)
            plot([0 tf],[0 0],'--k','linewidth',1)
            axis([0 tf -1.5 1.5])
            grid on
            xlabel('Time [s]')
            ylabel('u_{b3}')
            
            subplot(4,1,4)
            plot(sat.t,sat.J,'k','linewidth',1)
            axis([0 tf 0 1],'auto y')
            grid on
            xlabel('Time [s]')
            ylabel('Cost Function, J')
            drawnow
        end
        
        %% Plot fuel usage
        function plotFuel(sat)
            tf = sat.t(end);
            TP = sat.scenario.TP;
            
            figure
            plot(sat.t/TP,sat.fuel,'k','linewidth',1.25)
            axis([0 tf/TP 0 sat.fuel(1)])
            grid on
            xlabel('Time, Orbits','fontsize',12)
            ylabel('Fuel, kg','fontsize',12)
            drawnow
        end
        
        %% Display position on command window
        function printEphemeris(sat)
            clc
            fprintf('Time: %.2f\n',sat.t(end))
            fprintf('x: %6.3f m\n',sat.x(end))
            fprintf('y: %6.3f m\n',sat.y(end))
            fprintf('z: %6.3f m\n',sat.z(end))           
        end
        
        %% Record a video of figure
        function vid = renderVideo(sat,fileName,lbnd,ubnd,unit)
            close all
            fig = figure('Position',[100 100 1280 720]);
            
            satIter = newSatellite;
            vid = VideoWriter(fileName);
            vid.FrameRate = 30;
            vid.Quality = 100;
            open(vid);
            
            step = 15;
            for ii = 1:step:length(sat.x)
                clf
                satIter.x = sat.x(1:ii);
                satIter.y = sat.y(1:ii);
                satIter.z = sat.z(1:ii);
                satIter.ux = sat.ux(1:ii);
                satIter.uy = sat.uy(1:ii);
                satIter.uz = sat.uz(1:ii);
                satIter.q1 = sat.q1(ii);
                satIter.q2 = sat.q2(ii);
                satIter.q3 = sat.q3(ii);
                satIter.q4 = sat.q4(ii);
                satIter.plotTrajectory(lbnd,ubnd,unit);
                drawSatelliteX(1,4,[5,2],5);
                frame = getframe(fig);
                writeVideo(vid,frame);
            end
            close(vid);
            fprintf('Done\n')
        end
        
        %% Create STK scenario
        function createSTKobject(sat,app,root,model)
            sat.app = app;
            sat.root = root;
            createEphemerisFile(sat);  
            
            % Create satellite
            root.ExecuteCommand(sprintf('New / */Satellite %s',sat.name));
            efile = strcat(cd,'\',sat.name,'.e');
            afile = strcat(cd,'\',sat.name,'.a');
            root.ExecuteCommand(sprintf('SetState */Satellite/%s FromFile "%s"',sat.name,efile));
            root.ExecuteCommand(sprintf('SetAttitude */Satellite/%s File "%s"',sat.name,afile));
            root.ExecuteCommand(sprintf('VO */Satellite/%s Pass3D OrbitLead None OrbitTrail None',sat.name));
            
            switch model
                case '3U'
                    % Create 3U model
                    root.ExecuteCommand('VO */Satellite/RSO Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/cubesat_2u.dae"');
                    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Cubesat_2U Pitch 0 180',sat.name));
                    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Plus-Z-Panel Deploy -120 0',sat.name));
                    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Minus-Z-Panel Deploy 120 0',sat.name));
                    
                    % Keep deployables
                    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Plus-Y-Panel Deploy 120 45',sat.name));
                    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Minus-Y-Panel Deploy -120 -45',sat.name));
                case '6U'
                    % Create 6U model
                    root.ExecuteCommand(sprintf('VO */Satellite/%s Model File "C:/Program Files/AGI/STK 11/STKData/VO/Models/Space/cubesat_6u.dae"',sat.name));
                    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 6U-Cubesat Yaw 0 180',sat.name));
                    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Plus-Z-Plate Deploy 90 0',sat.name));
                    root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Minus-Z-Plate Deploy -90 0',sat.name));
                    
                    % Turn off Deployables
%                     root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Plus-Y-Plate Deploy -90 0',sat.name));
%                     root.ExecuteCommand(sprintf('VO */Satellite/%s Articulate "1 Jan 2000" 0 Minus-Y-Plate Deploy 90 0',sat.name));    
            end 
        end
        
        %% Power analysis
        function powerAnalysis(sat,model)
            if nargin < 2, model = '6U'; end
            
            % STK objects
            scenarioObj = sat.root.CurrentScenario;
            satObj = scenarioObj.Children.Item(sat.name);
            
            % Power analysis
            sat.root.UnitPreferences.Item('DateFormat').SetCurrentUnit('EpSec');
            sat.root.ExecuteCommand(sprintf('VO */Satellite/%s SolarPanel Compute "1 Jan 2018 00:00:00" "6 Jan 2018 00:00:00" 60',sat.name));
            powerDataProvider = satObj.DataProviders.Item('Solar Panel Power');
            powerDataProviderInterval = powerDataProvider.Exec(scenarioObj.StartTime,scenarioObj.StopTime,1);
            pTime = cell2mat(powerDataProviderInterval.DataSets.GetDataSetByName('Time').GetValues);
            pPaneldBW = cell2mat(powerDataProviderInterval.DataSets.GetDataSetByName('Power').GetValues);
            
            % Convert dBW to watts
            pPaneldBW = 10.^(pPaneldBW/10);
            
            % Interpolate to Matlab analysis time step
            time = sat.t;
            pPanel = interp1(pTime,pPaneldBW,time);
            
            %% Power usage
            switch model
                case '6U' % Active satellite
                    pADCSmin = 3.17;
                    pADCSmax = 7.23;
                    pPropmin = 0.25;
                    pPropmax = 10.00;
                    pCPU = 2*0.4;
                    pCam = 1.6;
                    pComm = 1.0;
                    pGPS = 0.8;
                    pIdle = pCPU+pCam+pComm+pGPS;
                    
                    eff = 1;
                    pPanel = pPanel*eff;
                    
                case '3U' % Target satellite
                    pADCSmin = 0;
                    pADCSmax = 0;
                    pPropmin = 0;
                    pPropmax = 0;
                    pIdle = 4;
                    
                    eff = 1.5;
                    pPanel = pPanel*eff;
            end
 
            % Calculate power usage
            pLoad = zeros(1,length(sat.t));
            for ii = 1:length(sat.t)         
                % Component power
                if sat.t(ii) < 3*sat.scenario.TP
                    pADCS = pADCSmin;
                else
                    pADCS = pADCSmin + (pADCSmax-pADCSmin)*...
                        sum(abs([sat.T1(ii),sat.T2(ii),sat.T3(ii)]/sat.Tmax));
                end
                pProp = pPropmin + (pPropmax-pPropmin)*...
                    sum(abs([sat.ux(ii),sat.uy(ii),sat.uz(ii)]/sat.umax));
                pLoad(ii) = pADCS+pProp+pIdle;
            end
            
            %% Energy Managment Utility                        
            % Constants
            k6 = -30.155;
            k5 = 97.5;
            k4 = -123.27;
            k3 = 77.946;
            k2 = -26.082;
            k1 = 4.8735;
            k0 = 3.1935;
            
            %% Initial conditions 40 Whr
            SOC40 = 0.5;                % State of charge (0-1)
            nSeries = 1;                % Number of modules
            nPacks = 1;                 % Number of battery packs
            ZbattCR = 0.1;              % BOL Battery Impedance (Ohm)
            ZbattDCR = 0.1;             % BOL Battery Impedance (Ohm)
            maxCR = 2.6*2*nPacks;       % Max Battery Charge Rate (A)
            maxDCR = -2*5.2*nPacks;     % Max Battery Discharge Rate (A)
            
            Emax = 40*3600;             % Battery capacity (J)
            Eavail = SOC40*Emax;
            Ibatt = 0;
            for ii = 1:(length(time)-1)
                % Available power
                pAvail(ii) = pPanel(ii)-pLoad(ii);
                
                if pAvail(ii) < 0
                    Zbatt = ZbattDCR; % Discharging
                else
                    Zbatt = ZbattCR; % Charging
                end
                
                % Battery voltage
                Vbatt_OC(ii) = nSeries*(k6*SOC40(ii)^6+k5*SOC40(ii)^5+k4*SOC40(ii)^4+...
                    k3*SOC40(ii)^3+k2*SOC40(ii)^2+k1*SOC40(ii)+k0);
                Vbatt(ii) = Vbatt_OC(ii)+Ibatt(ii)*Zbatt;
                
                % Available energy
                dt = time(ii+1)-time(ii);
                Eavail(ii+1) = Eavail(ii)+Ibatt(ii)*Vbatt(ii)*dt;
                if Eavail(ii+1) >= Emax
                    Eavail(ii+1) = Emax;
                elseif Eavail(ii+1) < 0
                    Eavail(ii+1) = 0;
                end
                
                % Available current
                Iavail = pAvail(ii)/Vbatt(ii);
                if Iavail >= maxCR
                    Ibatt(ii+1) = maxCR;
                elseif Iavail <= maxDCR
                    Ibatt(ii+1) = maxDCR;
                else
                    Ibatt(ii+1) = Iavail;
                end
                
                % State of charge
                SOC40(ii+1) = Eavail(ii+1)/Emax;
            end
            
            %% Initial conditions 80 Whr
            SOC80 = 0.5;                % State of charge (0-1)
            nSeries = 1;                % Number of modules
            nPacks = 2;                 % Number of battery packs
            ZbattCR = 0.1;              % BOL Battery Impedance (Ohm)
            ZbattDCR = 0.1;             % BOL Battery Impedance (Ohm)
            maxCR = 2.6*2*nPacks;       % Max Battery Charge Rate (A)
            maxDCR = -2*5.2*nPacks;     % Max Battery Discharge Rate (A)
            
            Emax = 80*3600;             % Battery capacity (J)
            Eavail = SOC80*Emax;
            Ibatt = 0;
            for ii = 1:(length(time)-1)
                % Available power
                pAvail(ii) = pPanel(ii)-pLoad(ii);
                
                if pAvail(ii) < 0
                    Zbatt = ZbattDCR; % Discharging
                else
                    Zbatt = ZbattCR; % Charging
                end
                
                % Battery voltage
                Vbatt_OC(ii) = nSeries*(k6*SOC80(ii)^6+k5*SOC80(ii)^5+k4*SOC80(ii)^4+...
                    k3*SOC80(ii)^3+k2*SOC80(ii)^2+k1*SOC80(ii)+k0);
                Vbatt(ii) = Vbatt_OC(ii)+Ibatt(ii)*Zbatt;
                
                % Available energy
                dt = time(ii+1)-time(ii);
                Eavail(ii+1) = Eavail(ii)+Ibatt(ii)*Vbatt(ii)*dt;
                if Eavail(ii+1) >= Emax
                    Eavail(ii+1) = Emax;
                elseif Eavail(ii+1) < 0
                    Eavail(ii+1) = 0;
                end
                
                % Available current
                Iavail = pAvail(ii)/Vbatt(ii);
                if Iavail >= maxCR
                    Ibatt(ii+1) = maxCR;
                elseif Iavail <= maxDCR
                    Ibatt(ii+1) = maxDCR;
                else
                    Ibatt(ii+1) = Iavail;
                end
                
                % State of charge
                SOC80(ii+1) = Eavail(ii+1)/Emax;
            end
            
            %% Initial conditions 120 Whr
            SOC120 = 0.5;                % State of charge (0-1)
            nSeries = 1;                % Number of modules
            nPacks = 3;                 % Number of battery packs
            ZbattCR = 0.1;              % BOL Battery Impedance (Ohm)
            ZbattDCR = 0.1;             % BOL Battery Impedance (Ohm)
            maxCR = 2.6*2*nPacks;       % Max Battery Charge Rate (A)
            maxDCR = -2*5.2*nPacks;     % Max Battery Discharge Rate (A)
            
            Emax = 120*3600;             % Battery capacity (J)
            Eavail = SOC120*Emax;
            Ibatt = 0;
            for ii = 1:(length(time)-1)
                % Available power
                pAvail(ii) = pPanel(ii)-pLoad(ii);
                
                if pAvail(ii) < 0
                    Zbatt = ZbattDCR; % Discharging
                else
                    Zbatt = ZbattCR; % Charging
                end
                
                % Battery voltage
                Vbatt_OC(ii) = nSeries*(k6*SOC80(ii)^6+k5*SOC120(ii)^5+k4*SOC120(ii)^4+...
                    k3*SOC120(ii)^3+k2*SOC120(ii)^2+k1*SOC120(ii)+k0);
                Vbatt(ii) = Vbatt_OC(ii)+Ibatt(ii)*Zbatt;
                
                % Available energy
                dt = time(ii+1)-time(ii);
                Eavail(ii+1) = Eavail(ii)+Ibatt(ii)*Vbatt(ii)*dt;
                if Eavail(ii+1) >= Emax
                    Eavail(ii+1) = Emax;
                elseif Eavail(ii+1) < 0
                    Eavail(ii+1) = 0;
                end
                
                % Available current
                Iavail = pAvail(ii)/Vbatt(ii);
                if Iavail >= maxCR
                    Ibatt(ii+1) = maxCR;
                elseif Iavail <= maxDCR
                    Ibatt(ii+1) = maxDCR;
                else
                    Ibatt(ii+1) = Iavail;
                end
                
                % State of charge
                SOC120(ii+1) = Eavail(ii+1)/Emax;
            end
            
            %% Plots
            TP = sat.scenario.TP;
            
            figure('Position',[100,100,1280,720])
            subplot(2,1,1)
            hold on
            plot(time/TP,pPanel,'LineWidth',1.5)
            plot(time/TP,pLoad,'LineWidth',1.5)
            plot(time/TP,pPanel-pLoad,'LineWidth',1.5)
            hold off
            grid on
            axis([0 time(end)/TP 0 1],'auto y')
            xlabel('Time, Orbits','FontSize',12)
            ylabel('Watts','FontSize',12)
            legend({'Solar Power Generation','Power Usage','Net Power'},...
                'location','northoutside','Orientation','horizontal','FontSize',12)
            
            subplot(2,1,2)
            hold on
            plot(time/TP,SOC40*100,'LineWidth',1.5)
            plot(time/TP,SOC80*100,'LineWidth',1.5)
            plot(time/TP,SOC120*100,'LineWidth',1.5)
            hold off
            grid on
            axis([0 time(end)/TP 0 100])
            xlabel('Time, Orbits','FontSize',12)
            ylabel('State of Charge, %','FontSize',12)
            legend({'40 Whr Battery','80 Whr Battery'},'location','northoutside','Orientation','horizontal','FontSize',12)
            legend({'40 Whr Battery','80 Whr Battery','120 Whr Battery'},...
                'location','northoutside','Orientation','horizontal','FontSize',12)           
            drawnow
        end
    end
end

%% Local MILP functions
% Functions used to set up A,b,Aeq, & beq matrices for MILP optimization
% problems.

% Add obstacle integer constraints
function [Anew,bnew] = addObstacle(Aold,bold,sat,lbnd,ubnd,N)
R = sat.Rib;

dt = sat.scenario.dt;
Nsim = sat.Nsim;
Nvar = sat.Nvar;
Neom = sat.Neom;
Ntotal = sat.Ntotal;
Nslack = sat.Nslack;
beta = sat.umax*dt^2/sat.m; %velocity multiplier
M = 1e6;

% Safety buffer
lbnd = lbnd-sat.dsafe;
ubnd = ubnd+sat.dsafe;

% Add inequality contraints
A = zeros(Nsim*7,Ntotal);
b = zeros(Nsim*7,1);
for ii = 1:7:(7*Nsim) %counting position of rows (+/-x,y,z...binary)
    jj = 1;           %counting position of columns (in u())
    
    nn = (ii+6)/7;
    for kk = 1:nn      %counting no. iterations
        A(ii:ii+2,jj:2:jj+5) = R*beta*(nn-kk);
        A(ii:ii+2,jj+1:2:jj+5) = -R*beta*(nn-kk);
        if kk > 1
            A(ii:ii+2,Nvar+3*(kk-2)+1:Nvar+3*(kk-2)+3) = R*dt^2*(nn-kk+1);
        end
        A(ii+3:ii+5,:) = -A(ii:ii+2,:);
        jj = jj+6;
    end
    
    % Binary variables
    jj = Nvar+Neom+Nslack+Nvar*(N-1)+6*(nn-1)+1;
    A(ii:ii+5,jj:jj+5) = -M*eye(6);
    A(ii+6,jj:jj+5) = ones(1,6);
    
    b(ii:ii+2) = (lbnd-sat.p)-nn*dt*sat.v;
    b(ii+3:ii+5) = -((ubnd-sat.p)-nn*dt*sat.v);
    b(ii+6) = 5;

end

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end

% Hold constraints
function [Anew,bnew] = holdProximity(Aold,bold,sat,lbnd,ubnd)
R = sat.Rib;
dt = sat.scenario.dt;
Nsim = sat.Nsim;
Nvar = sat.Nvar;
Ntotal = sat.Ntotal;
beta = sat.umax*dt^2/sat.m; %velocity multiplier

% Add inequality contraints
A = zeros(6*Nsim,Ntotal);
b = zeros(6*Nsim,1);
for ii = 1:6:(6*Nsim); %counting position of rows (+/-x,y,z)
    jj = 1;     
    nn =(ii+5)/6;
    for kk = 1:nn
        A(ii:ii+2,jj:2:jj+5) = R*beta*(nn-kk);
        A(ii:ii+2,jj+1:2:jj+5) = -R*beta*(nn-kk);    
        
        % Equations of motion
        if kk > 1
            A(ii:ii+2,Nvar+3*(kk-2)+1:Nvar+3*(kk-2)+3) = R*dt^2*(nn-kk+1);
            A(ii+3:ii+5,Nvar+3*(kk-2)+1:Nvar+3*(kk-2)+3) = -R*dt^2*(nn-kk+1);
        end      
        jj = jj+6;
    end
    A(ii+3:ii+5,:) = -A(ii:ii+2,:);
    
    b(ii:ii+2) = (ubnd-sat.p)-nn*dt*sat.v;
    b(ii+3:ii+5) = -((lbnd-sat.p)-nn*dt*sat.v);    
end

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end

% Velocity constraints
function [Anew,bnew] = maxVelocity(Aold,bold,sat)
R = sat.Rib;
dt = sat.scenario.dt;
Nsim = sat.Nsim;
Nvar = sat.Nvar;
Ntotal = sat.Ntotal;
alpha = sat.umax*dt/sat.m;

A = zeros(6*Nsim,Ntotal);
b = zeros(6*Nsim,1);
for ii = 1:6:(6*Nsim) 
    jj = 1;           
    for kk = 1:(ii+5)/6     
        % Postive bounds    
        A(ii:ii+2,jj:2:jj+5) = R*alpha;
        A(ii:ii+2,jj+1:2:jj+5) = -R*alpha;
        % EOM
        if kk > 1
            A(ii:ii+2,Nvar+3*(kk-2)+1:Nvar+3*(kk-2)+3) = R*dt;
        end
        
        % Negative Bound
        A(ii+3:ii+5,:) = -A(ii:ii+2,:);
        jj = jj+6;
    end
    
    % Set equivalence 
    b(ii:ii+2) = (sat.vmax-sat.v)';
    b(ii+3:ii+5) = (sat.vmax+sat.v)';
end

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end

% Approach slack variables
function [Anew,bnew] = minDistance(Aold,bold,sat,pf)
% Approach target cost function contraints
dt = sat.scenario.dt;
R = sat.Rib;
Nsim = sat.Nsim;
Nvar = sat.Nvar;
Neom = sat.Neom;
Nslack = sat.Nslack;
Ntotal = sat.Ntotal;
beta = sat.umax*dt^2/sat.m; %Velocity multiplier

% Target objective absolute value
A = zeros(6,Ntotal);
b = zeros(6,1);
      

for ii = 1:Nsim 
    jj = 6*ii-5;
    % Positive bound
    A(1:3,jj:2:jj+5) = R*beta*(Nsim-ii);
    A(1:3,jj+1:2:jj+5) = -R*beta*(Nsim-ii);
    
    % Equations of motion
    if ii > 1
        A(1:3,Nvar+3*(ii-2)+1:Nvar+3*(ii-2)+3) = R*dt^2*(Nsim-ii+1);
    end       
end
% Negative bound
A(4:6,:) = -A(1:3,:);

% Set equivalence
A(1:6,Nvar+Neom+1:Nvar+Neom+Nslack) = -[eye(3);eye(3)];

% Initial conditions
b(1:3) = (-sat.p+pf-Nsim*dt*sat.v)';
b(4:6) = -(-sat.p+pf-Nsim*dt*sat.v)';

% Update matrices
Anew = [Aold;A];
bnew = [bold;b];
end

% Solve for equations of motion
function [Aeqnew,beqnew] = setEOM(Aeqold,beqold,sat,R)
if nargin < 4 || isempty(R),R = sat.Rib;end  
dt = sat.scenario.dt;
Nsim = sat.Nsim;
Nvar = sat.Nvar;
Ntotal = sat.Ntotal;
alpha = sat.umax*dt/sat.m;  %position multiplier
beta = sat.umax*dt^2/sat.m; %velocity multiplier
t = sat.t(end);

% Add inequality contraints
Aeq = zeros(3*Nsim,Ntotal);
beq = zeros(3*Nsim,1);
for ii = 1:3:(3*Nsim) 
    nn = (ii+2)/3;
    
    % Define equtions of motion
    switch sat.EOM
        case 'HCW'
            A = HCW(sat.scenario);
        case 'LERM'
            A = LERM(sat.scenario,t);
    end
    C = A(4:6,:);
    
    jj = 1;
    for kk = 1:nn
        % Solve for state at each time step
        Aeq(ii:ii+2,jj:2:jj+5) = R*C(:,1:3)*beta*(nn-kk)+R*C(:,4:6)*alpha;
        Aeq(ii:ii+2,jj+1:2:jj+5) = -R*C(:,1:3)*beta*(nn-kk)-R*C(:,4:6)*alpha;
        
        % Set equivalence
        Aeq(ii:ii+2,Nvar+3*(nn-1)+1:Nvar+3*(nn-1)+3) = -eye(3);
        
        % Equations of motions
        if kk > 1            
            Aeq(ii:ii+2,Nvar+3*(kk-2)+1:Nvar+3*(kk-2)+3) = ...
                R*C(:,1:3)*(dt^2*(nn-kk+1))+R*C(:,4:6)*dt;
        end
        jj = jj+6;
    end 
    
    % Initial Conditions
    beq(ii:ii+2) = -C(:,1:3)*(sat.p+nn*dt*sat.v)'-C(:,4:6)*(sat.v)';
        
    % Update time
    t = t+dt;
end

% Update matrices
Aeqnew = [Aeqold;Aeq];
beqnew = [beqold;beq];
end

% Long range eqaulity constraints
function [Aeqnew,beqnew] = setPhaseStateEq(Aeqold,beqold,sat,xf)
dt = sat.scenario.dt;
R = eye(3);
Nsim = sat.Nsim;
Nvar = sat.Nvar;
Ntotal = sat.Ntotal;
alpha = sat.umax*dt/sat.m;
beta = sat.umax*dt^2/sat.m; %Velocity multiplier

% Target objective absolute value
Aeq = zeros(6,Ntotal);
beq = zeros(6,1);

for ii = 1:Nsim 
    jj = 6*ii-5;
    % Positive bound
    Aeq(1:3,jj:2:jj+5) = R*beta*(Nsim-ii);
    Aeq(1:3,jj+1:2:jj+5) = -R*beta*(Nsim-ii);
    Aeq(4:6,jj:2:jj+5) = R*alpha;
    Aeq(4:6,jj+1:2:jj+5) = -R*alpha;
    
    % Equations of motion
    if ii > 1
        Aeq(1:3,Nvar+3*(ii-2)+1:Nvar+3*(ii-2)+3) = R*dt^2*(Nsim-ii+1);
        Aeq(4:6,Nvar+3*(ii-2)+1:Nvar+3*(ii-2)+3) = R*dt;
    end       
end

% Initial conditions
beq(1:3) = (xf(1:3)-sat.p-Nsim*dt*sat.v)';
beq(4:6) = (xf(4:6)-sat.v)';


% Update matrices
Aeqnew = [Aeqold;Aeq];
beqnew = [beqold;beq];
end

%% Other local functions
% Rotation matrices
function R = rot(theta,axis)
% Rotation matrices
switch axis
    case 1
        R = [1 0           0
            0 cos(theta) -sin(theta)
            0 sin(theta)  cos(theta)];
    case 2
        R = [cos(theta) 0 sin(theta)
            0          1 0
            -sin(theta) 0 cos(theta)];
    case 3
        R = [cos(theta) -sin(theta) 0
            sin(theta)  cos(theta) 0
            0           0          1];
end
end

% Hill-Clohessy-Wiltshire equations
function A = HCW(scenario)
n = scenario.n;
A = [zeros(3,3) eye(3)
    3*n^2 0 0 0 2*n 0
    0 0 0 -2*n 0 0
    0 0 -n^2 0 0 0];
end

% Linear equtations of relative motion
function A = LERM(scenario,t)
% Orginal Author: Andrew C. Rogers
mu = scenario.mu;
a = scenario.a;
ecc = scenario.ecc;
nu = scenario.nu;

% Time varying coefficients and solution to Kepler's equation
n = sqrt(mu/a^3);
t0 = nu2t(n,nu,ecc);
M = n.*(t+t0);
[EccAnom,~] = kepler(M,ecc);
f = 2.*atan(sqrt((1+ecc)./(1-ecc)).*tan(EccAnom./2));
r = (a.*(1-ecc.^2))./(1+ecc.*cos(f));
fdot = sqrt(mu.*a.*(1-ecc.^2)).*(1+ecc.*cos(f)).^2./(a.^2.*(1-ecc.^2).^2);
rdot = ecc.*sin(f).*sqrt(mu.*a.*(1-ecc.^2))./(a.*(1-ecc.^2));
fddot = -2.*rdot.*fdot./r;

% State matrix for LERM
A = [0,0,0,1,0,0;
      0,0,0,0,1,0;
      0,0,0,0,0,1;
      (fdot^2+2*mu/r^3) fddot 0 0 2*fdot 0;
      -fddot (fdot^2-mu/r^3) 0 -2*fdot 0 0
      0 0 -mu/r^3 0 0 0];
end

% Kepler's equation
function [E,iter] = kepler(M,ecc)
% Function solves Kepler's equation:
% M = n*(t-t_0) = E-e*sin(E)
% Using Newton-Raphson iteration
% AC Rogers, 21 February 2013
% Inputs:
%           M    = mean anomaly
%           e    = eccentricity
% Outputs:
%           E    = eccentric anomaly
%           iter = number of iterations
tol = 1e-12;
iter = 1;
for ii = 1:length(M)
    E_n = M(ii);
    f_n = E_n-ecc.*sin(E_n) - M(ii);
    while (abs(f_n) > tol)
        E_n = E_n - f_n./(1-ecc.*cos(E_n));
        f_n = E_n - ecc.*sin(E_n) - M(ii);
        iter = iter + 1;
    end
    E(ii) = E_n;
end
end

% Time from periapsis
function t = nu2t(n,nu,ecc)
% Function solves for time from periapsis given mean motion, ecentricity,
% and true anomaly
% IE, 08/30/2016
E = acos((ecc+cosd(nu))/(1+ecc*cosd(nu)));
M = E-ecc*sin(E);
t = M/n;
end

% Plot obstacle
function p = plotObstacle(lbnd,ubnd,style)
xbnd = [lbnd(1) ubnd(1) ubnd(1) lbnd(1) lbnd(1)];
ybnd = [lbnd(2) lbnd(2) ubnd(2) ubnd(2) lbnd(2)];
zlbnd = [lbnd(3) lbnd(3) lbnd(3) lbnd(3) lbnd(3)];
zubnd = [ubnd(3) ubnd(3) ubnd(3) ubnd(3) ubnd(3)];

hold on
p = plot3(xbnd,ybnd,zlbnd,style,'linewidth',1);
plot3(xbnd,ybnd,zubnd,style,'linewidth',1)
for ii = 1:4
    plot3([xbnd(ii) xbnd(ii)],[ybnd(ii),ybnd(ii)],[lbnd(3) ubnd(3)],style,'linewidth',1)
end
end

% Draw satellite
function plotSatellite(r,h,panel,panelAngle)
grid on

c = [0.5,0.5,0.5];
ang = pi/8;
theta = linspace(ang,2*pi+ang);

% Top face
topy = r*cos(theta);
topz = r*sin(theta);
topx = h/2*ones(1,length(theta));
patch(topx,topy,topz,c,'EdgeColor','None')

% Bottom face
boty = r*cos(theta);
botz = r*sin(theta);
botx = -h/2*ones(1,length(theta));

patch(botx,boty,botz,c,'EdgeColor','None')

% Panels
panc = [0.6,0.8,1];
panz = [r,r+panel(1),r+panel(1),r];
pany = [-panel(2)/2,-panel(2)/2,panel(2)/2,panel(2)/2]*cosd(panelAngle);
panx = [-panel(2)/2,-panel(2)/2,panel(2)/2,panel(2)/2]*sind(panelAngle);
patch(panx,pany,panz,panc)
patch(panx,pany,-panz,panc)

% Sides
for ii = 1:length(theta)-1
   x = [topx(ii),topx(ii+1),botx(ii+1),botx(ii)];
   y = [topy(ii),topy(ii+1),boty(ii+1),boty(ii)];
   z = [topz(ii),topz(ii+1),botz(ii+1),botz(ii)];
   patch(x,y,z,c,'EdgeColor','None')
end

% Plotting
% light('Position',[0 -100 100],'Style','local');
end

function plotShuttle(x,y,z,pitch,roll,yaw,scale_factor,step,cv)
load shuttle;
V = [-V(:,2) V(:,1) V(:,3)];
V(:,1) = V(:,1)-round(sum(V(:,1))/size(V,1));
V(:,2) = V(:,2)-round(sum(V(:,2))/size(V,1));
V(:,3) = V(:,3)-round(sum(V(:,3))/size(V,1));

correction = max(abs(V(:,1)));
V = V./(scale_factor*correction);
ii = length(x);
resto = mod(ii,step);

y = y;
z = z;
pitch = pitch;
roll = roll;
yaw = -yaw;

for jj = 1:step:(ii-resto)
    theta = pitch(jj);
    phi = -roll(jj);
    psi = yaw(jj);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Tbe = [cos(psi)*cos(theta), -sin(psi)*cos(theta), sin(theta);
        cos(psi)*sin(theta)*sin(phi)+sin(psi)*cos(phi) ...
        -sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) ...
        -cos(theta)*sin(phi);
        -cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi) ...
        sin(psi)*sin(theta)*cos(phi)+cos(psi)*sin(phi) ...
        cos(theta)*cos(phi)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Vnew = V*Tbe;
    rif = [x(jj) y(jj) z(jj)];
    X0 = repmat(rif,size(Vnew,1),1);
    Vnew = Vnew + X0;
    p = patch('faces', F, 'vertices' ,Vnew);
    set(p, 'facec', cv);
    set(p, 'EdgeColor','none');
    H1 = light('Position',[100 0 0],'Style','local');
    camva(9)
    hold on
%     lighting phong
    daspect([1 1 1]) ;
    
end
end

% Dormand - Prince Runge Kutta integration
function X = RKDP(odefun,t0,dt,X0,sat,u)
% Step 1
X1    = X0;
dXdt1 = odefun(t0,X1,sat,u);

% Step 2
X2    = X0 + (1/5)*dXdt1*dt;
dXdt2 = odefun(t0+(1/5)*dt,X2,sat,u);

% Step 3
X3    = X0 + ((3/40)*dXdt1+(9/40)*dXdt2)*dt;
dXdt3 = odefun(t0+(3/10)*dt,X3,sat,u);

% Step 4
X4    = X0 + ((44/45)*dXdt1-(56/15)*dXdt2+(32/9)*dXdt3)*dt;
dXdt4 = odefun(t0+(4/5)*dt,X4,sat,u);

% Step 5
X5    = X0 + ((19372/6561)*dXdt1+-(25360/2187)*dXdt2+(64448/6561)*dXdt3...
        -(212/729)*dXdt4)*dt;
dXdt5 = odefun(t0+(8/9)*dt,X5,sat,u);

% Step 6
X6    = X0 + ((9017/3168)*dXdt1-(355/33)*dXdt2+(46732/5247)*dXdt3...
        +(49/176)*dXdt4-(5103/18656)*dXdt5)*dt;
dXdt6 = odefun(t0+dt,X6,sat,u);

% Step 7
X7    = X0 + ((35/384)*dXdt1+0*dXdt2+(500/1113)*dXdt3...
        +(125/192)*dXdt4-(2187/6784)*dXdt5+(11/84)*dXdt6)*dt;
dXdt7 = odefun(t0+dt,X7,sat,u);

% New state
X = X7;

% 4th order solution
% X4th     = X0 + ((5179/57600)*dXdt1+(7571/16695)*dXdt3+(393/640)*dXdt4...
%         -(92097/339200)*dXdt5+(187/2100)*dXdt6+(1/40)*dXdt7)*dt;
end

% Attitude standard ode setup
function dXdt = attitudeODE(~,X,sat,u)
dXdt = zeros(size(X));
I = sat.I;

w1 = X(1);
w2 = X(2);
w3 = X(3);
q1 = X(4);
q2 = X(5);
q3 = X(6);
q4 = X(7);
w = [w1;w2;w3];
q = [q1;q2;q3];

% Angular velocity
dXdt(1) = (I(2)-I(3))/I(1)*w2*w3 + u(1)/I(1);
dXdt(2) = (I(3)-I(1))/I(2)*w1*w3 + u(2)/I(2);
dXdt(3) = (I(1)-I(2))/I(3)*w1*w2 + u(3)/I(3);

% Quaternions
qx = [0  -q3  q2
      q3  0  -q1
     -q2  q1  0 ];
dqdt = 1/2*[qx+q4*eye(3);-q']*w;

dXdt(4) = dqdt(1);
dXdt(5) = dqdt(2);
dXdt(6) = dqdt(3);
dXdt(7) = dqdt(4);
end

% Translation standard ode setup
function dXdt = trajectoryODE(t,X,sat,u)
A = LERM(sat.scenario,t);
dXdt = A*X+u/sat.m;
end

% Create STK ephemeris files
function createEphemerisFile(sat)
% Trajectory
filename = strcat(sat.name,'.e');
fileID = fopen(filename,'w');
fprintf(fileID,'stk.v.8.0\r\n\r\nBEGIN Ephemeris\r\n\r\n');
fprintf(fileID,'NumberOfEphemerisPoints %d\r\n',length(sat.x));
fprintf(fileID,'CoordinateSystem Custom RIC Satellite/Origin\r\n\r\n');
fprintf(fileID,'EphemerisTimePosVel\r\n\r\n');

for ii = 1:length(sat.x);
    fprintf(fileID,'%f %f %f %f %f %f %f\r\n',sat.t(ii),sat.x(ii),sat.y(ii),sat.z(ii),...
        sat.vy(ii),-sat.vz(ii),-sat.vx(ii));
end

fprintf(fileID,'END Ephemeris');
fclose(fileID);

% Attitude
filename = strcat(sat.name,'.a');
fileID = fopen(filename,'w');
fprintf(fileID,'stk.v.8.0\r\n\r\nBEGIN Attitude\r\n\r\n');
fprintf(fileID,'NumberOfAttitudePoints %d\r\n',length(sat.x));
fprintf(fileID,'CoordinateSystem Custom RIC Satellite/Origin\r\n\r\n');
% fprintf(fileID,'AttitudeTimeQuatAngVels\r\n\r\n');
fprintf(fileID,'AttitudeTimeQuaternions\r\n\r\n');

for ii = 1:length(sat.x);
    fprintf(fileID,'%f %f %f %f %f\r\n',sat.t(ii),sat.q1(ii),sat.q2(ii),sat.q3(ii),sat.q4(ii));
end

fprintf(fileID,'END Attitude');
fclose(fileID);
end