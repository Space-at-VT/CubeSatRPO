%% New Satellite
classdef newSatellite < handle
    %% Default properties of a basic 6u cubesat
    properties
        %% Model
        name = 'CubeSat'            %Satellite name
        EOM = 'LERM'                %Relative motion model
        mode = 'approach'           %Satellite objective
        color = 'b'                 %Graph color
        
        %% Scenario
        scenario = newScenario
        
        %% Satellite parameters
        umax = 0.25                 %Thrust,                N
        Isp = 150                   %Specific impulse,      s
        dryMass = 13                %Dry mass,              kg
        fuel = 0.5                  %Fuel mass,             kg
        vmax = 0.5                  %Max velocity,          m/s
        bnd = [0.1,0.3,0.2]         %Satellite size,        m
        Tmax = 0.007                %Max reaction torque    Nm
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
        T1 = []                     %x reaction torque,     N*m
        T2 = []                     %y reaction torque,     N*m
        T3 = []                     %z reaction torque,     N*m
        
        %% Attitude
        wb1 = 0                     %Angular velocity       rad/s
        wb2 = 0                     %over time              rad/s
        wb3 = 0                     %                       rad/s
        q1 = 0                      %Quaternions
        q2 = 0                      %over time
        q3 = 0                      %
        q4 = 1;                     %Quaternion scalar
        point = 0                   %Attitude pointing      binary
        pt = [0,0,0]                %Attitude point target  m
        
        %% Solver
        T = 15
        Nobj = 0
        Nslack = 0
        u = []                      %Current u output
        eom = []                    %Current state ouput
        J = 0                       %Cost fucntion over time
        flag = []                   %Exit flag
        
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
        
        %% Attitude
        w                           %Angular velocity,      rad/s
        qb                          %Quaternions vector,
        Rib                         %Rotation matrix b2i
        Rbi                         %Rotation matrix i2b
        ubnd                        %Upper bound,           m
        lbnd                        %Lower bound,           m
        I                           %Moments of Inertia     kg/m^2
        dv_ideal                    %Theoretical total dv   m/s
        
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
                intcon = [1:sat.Nvar,sat.Nvar+sat.Neom+3+1:sat.Ntotal];
                
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
                intcon = 1:sat.Nvar;
                
                % Equality contraints
                Aeq = []; beq = [];
                [Aeq,beq] = setEOM(Aeq,beq,sat);
                
                % Inequality contraints
                A = [];   b = [];
                [A,b] = holdProximity(A,b,sat,lbnd,ubnd);
                [A,b] = maxVelocity(A,b,sat);
                
                options = optimoptions(@intlinprog,'Display','None','MaxTime',dt);
                [U,fval,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub,options);
                
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
            f = [dtM*ones(Nvar,1)           %Control thrusts
                 zeros(Neom,1)];
            
            % Parameter bounds, lower & upper
            lb = [zeros(Nvar,1)             %Control thrusts
                 -inf*ones(Neom,1)];
            ub = [ones(Nvar,1);             %Control thrusts
                  inf*ones(Neom,1)];
            
            % Equality contraints
            Aeq = []; beq = [];
            [Aeq,beq] = setEOM(Aeq,beq,sat,eye(3));
            [Aeq,beq] = setPhaseStateEq(Aeq,beq,sat,Xf);
            
            % Inequality contraints
            A = [];   b = [];
            
            % Integer constraint
            intcon = [];
            
            options = optimoptions(@intlinprog,'MaxTime',10);%'Display','None',
            [U,fval,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub);
            
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
                sat.flag(iter) = 0;
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
            
            % Runge-kutta
%             [DV,DX] = rk4(sat);
%             sat.vx(iter+1) = sat.vx(iter)+DV(1);
%             sat.vy(iter+1) = sat.vy(iter)+DV(2);
%             sat.vz(iter+1) = sat.vz(iter)+DV(3);
%             sat.x(iter+1) = sat.x(iter)+DX(1);
%             sat.y(iter+1) = sat.y(iter)+DX(2);
%             sat.z(iter+1) = sat.z(iter)+DX(3);
              
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
            
            % Fuel mass loss
            sat.dv(iter+1) = sat.dv(iter)+sum(abs(U(1:6)))*(sat.umax/sat.m)*dt;
            sat.fuel(iter+1) = sat.fuel(iter)-sum(abs(U(1:6)))*sat.mdot*dt;
        end
        
        %% Propagate trajectory with no fuel
        function sat = driftProp(sat)
            dt = sat.scenario.dt;
            
            % Switch equations of motion model state matrix
            switch sat.EOM
                case 'HCW'
                    A = HCW(sat.scenario);
                case 'LERM'
                    A = LERM(sat.scenario,sat.t(end));
            end
            
            % Calculate new state
            iter = length(sat.x);
            sat.flag(iter) = 3;
            
            X = [sat.x(iter),sat.y(iter),sat.z(iter),...
                 sat.vx(iter),sat.vy(iter),sat.vz(iter)]';
            DX = A*X;

            % Control signals - all off
            sat.ub1(iter) = 0;
            sat.ub2(iter) = 0;
            sat.ub3(iter) = 0;   
            sat.ux(iter) = 0;
            sat.uy(iter) = 0;
            sat.uz(iter) = 0;
            
            sat.vx(iter+1) = sat.vx(iter)+DX(4)*dt;
            sat.vy(iter+1) = sat.vy(iter)+DX(5)*dt;
            sat.vz(iter+1) = sat.vz(iter)+DX(6)*dt;
            sat.x(iter+1) = sat.x(iter)+sat.vx(iter)*dt;
            sat.y(iter+1) = sat.y(iter)+sat.vy(iter)*dt;
            sat.z(iter+1) = sat.z(iter)+sat.vz(iter)*dt;


%             [DV,DX] = rk4(sat);
%             sat.vx(iter+1) = sat.vx(iter)+DV(1);
%             sat.vy(iter+1) = sat.vy(iter)+DV(2);
%             sat.vz(iter+1) = sat.vz(iter)+DV(3);
%             sat.x(iter+1) = sat.x(iter)+DX(1);
%             sat.y(iter+1) = sat.y(iter)+DX(2);
%             sat.z(iter+1) = sat.z(iter)+DX(3);
            
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
            if sat.point
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
            else
                sat.T1(iter) = -kp*(sat.q1(iter))-kd*sat.wb1(iter);
                sat.T2(iter) = -kp*(sat.q2(iter))-kd*sat.wb2(iter);
                sat.T3(iter) = -kp*(sat.q3(iter))-kd*sat.wb3(iter);
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
            
            % New angular velocity
            I = sat.I;
            w = sat.w;
            sat.wb1(iter+1) = sat.wb1(iter)+((I(2)-I(3))/I(1)*w(2)*w(3)+M(1)/I(1))*dt;
            sat.wb2(iter+1) = sat.wb2(iter)+((I(3)-I(1))/I(2)*w(1)*w(3)+M(2)/I(2))*dt;
            sat.wb3(iter+1) = sat.wb3(iter)+((I(1)-I(2))/I(3)*w(1)*w(2)+M(3)/I(3))*dt;
            
            % Solve for quaternion rate of change
            q = sat.qb(1:3);
            q4 = sat.qb(4);
            qx = [0   -q(3) q(2)
                  q(3) 0   -q(1)
                 -q(2) q(1) 0   ];
            dqdt = 1/2*[qx+q4*eye(3);-q']*w;
            
            % Normalize quaternions to account for numerical error (Ensures quaternions
            % don't diverge to unreal values.
            qb = sat.qb+dqdt*dt;
            qb = qb/norm(qb);
            
            % New quaternions
            sat.q1(iter+1) = qb(1);
            sat.q2(iter+1) = qb(2);
            sat.q3(iter+1) = qb(3);
            sat.q4(iter+1) = qb(4);
        end
        
%         %% Plot relative trajectory
%         function plotTrajectory(sat,lbnd,ubnd,unit)
%             if nargin < 4 || isempty(unit),unit = 5;end
%             figure(1);
%             
%             def = [0,0.4470,0.7410];
%             
%             % Plot obstacle bounds
%             for ii = 1:size(lbnd,1)
%                 bnds = plotObstacle(lbnd(ii,:),ubnd(ii,:),'-k');
%             end
%                        
%             % Plot satellite trajectory
%             hold on            
%             for jj = 1:length(sat)                
%                 p0 = plot3(sat.x(1),sat.y(1),sat.z(1),'ko','markerfacecolor','g','markersize',6);
%                 pf = plot3(sat.x(end),sat.y(end),sat.z(end),'ko','markerfacecolor','r','markersize',4);
%                 trj = plot3(sat.x,sat.y,sat.z,'-','linewidth',1,'color',def);
%                 %quiver3(sat.x,sat.y,sat.z,-sat.ux,-sat.uy,-sat.uz,1,'r','linewidth',1.5);
%                 
%                 % Plot satellite body axes
%                 R = unit*sat.Rib;
%                 plot3([sat.p(1),sat.p(1)+R(1,1)'],[sat.p(2),sat.p(2)+R(2,1)'],...
%                     [sat.p(3),sat.p(3)+R(3,1)'],'b','linewidth',1);
%                 plot3([sat.p(1),sat.p(1)+R(1,2)'],[sat.p(2),sat.p(2)+R(2,2)'],...
%                     [sat.p(3),sat.p(3)+R(3,2)'],'r','linewidth',1);
%                 plot3([sat.p(1),sat.p(1)+R(1,3)'],[sat.p(2),sat.p(2)+R(2,3)'],...
%                     [sat.p(3),sat.p(3)+R(3,3)'],'g','linewidth',1);
%             end
%             hold off
%             
%             % Axis labels
%             grid on
%             xlabel('Radial, x [m]')
%             ylabel('In-track, y [m]')
%             zlabel('Cross-track, z [m]')
%             axis('tight','equal','vis3d')
%             legend([bnds,p0,trj],{'Chief Bounds',...
%                 'Initial Position','Relative Trajectory'},'location','best')
%             camva(8)
%             view(30,20)        
%             drawnow
%         end
%         
        %% Plot relative trajectory in multiple frames
        function subplotTrajectory(sat)
            
            figure('Position',[100 100 1280 720]);
         
            % 3D view
            axes('Position',[0.05 0.1 0.5 0.8]);
            axis('equal')
            camva(9)
            hold on
            plot3(sat.x,sat.y,sat.z,'linewidth',1.5);
            quiver3(sat.x,sat.y,sat.z,-sat.ux,-sat.uy,-sat.uz,5,'r','linewidth',1.5);
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
            plot3(sat.x,sat.y,sat.z,'linewidth',1.5);
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
            plot3(sat.x,sat.y,sat.z,'linewidth',1.5);
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
            plot3(sat.x,sat.y,sat.z,'linewidth',1.5);
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
            
        end
        
        %% Plot control signals and state over time
        function plotControls(sat)
            tf = sat.t(end);
            
            c1 = [0,0.4470,0.7410];
            c2 = [0.8500,0.3250,0.0980];
            c3 = [0.9290,0.6940,0.1250];
            
            % main view
            figure
            subplot(3,3,1)
            hold on
            stairs(sat.t,sat.ub1/sat.umax,'-b','linewidth',2)
            plot([0 tf],[0 0],'--k','linewidth',2)
            axis([0 tf -1.5 1.5])
            grid on
            title('Control Signals vs Time')
            ylabel('ub1')
            
            subplot(3,3,4)
            hold on
            stairs(sat.t,sat.ub2/sat.umax,'-r','linewidth',2)
            plot([0 tf],[0 0],'--k','linewidth',2)
            axis([0 tf -1.5 1.5])
            grid on
            ylabel('ub2')
            
            subplot(3,3,7)
            hold on
            stairs(sat.t,sat.ub3/sat.umax,'-g','linewidth',2)
            plot([0 tf],[0 0],'--k','linewidth',2)
            axis([0 tf -1.5 1.5])
            grid on
            xlabel('Time [s]')
            ylabel('ub3')
            
            % Velocity
            subplot(3,3,2)
            plot(sat.t,sat.vx,'-b','linewidth',2)
            axis([0 tf 0 1],'auto y')
            grid on
            ylabel('Vx [m/s]')
            title('Velocity vs Time')
            
            subplot(3,3,5)
            plot(sat.t,sat.vy,'-r','linewidth',2)
            axis([0 tf 0 1],'auto y')
            grid on
            ylabel('Vy [m/s]')
            
            subplot(3,3,8)
            plot(sat.t,sat.vz,'-g','linewidth',2)
            axis([0 tf 0 1],'auto y')
            grid on
            xlabel('Time [s]')
            ylabel('Vz [m/s]')
            
            % Position
            subplot(3,3,3)
            plot(sat.t,sat.x,'-b','linewidth',2)
            axis([0 tf 0 1],'auto y')
            grid on
            ylabel('x [m]')
            title('Position vs Time')
            
            subplot(3,3,6)
            plot(sat.t,sat.y,'-r','linewidth',2)
            axis([0 tf 0 1],'auto y')
            grid on
            ylabel('y [m]')
            
            subplot(3,3,9)
            plot(sat.t,sat.z,'-g','linewidth',2)
            axis([0 tf 0 1],'auto y')
            grid on
            xlabel('Time [s]')
            ylabel('z [m]')
            
            %
            figure
            subplot(3,1,1)
            hold on
            plot(sat.t,sat.T1,'color',c1,'linewidth',1.5)
            plot(sat.t,sat.T2,'color',c2,'linewidth',1.5)
            plot(sat.t,sat.T3,'color',c3,'linewidth',1.5)
            hold off
            axis([0 tf 0 1],'auto y')
            grid on
            legend({'T1','T2','T3'})
            xlabel('Time [s]')
            ylabel('Reaction Torques, Nm')
            title('Reaction Torques vs Time')
            
            subplot(3,1,2)
            hold on
            plot(sat.t,sat.wb1,'color',c1,'linewidth',1.5)
            plot(sat.t,sat.wb2,'color',c2,'linewidth',1.5)
            plot(sat.t,sat.wb3,'color',c3,'linewidth',1.5)
            hold off
            axis([0 tf 0 1],'auto y')
            grid on
            legend({'\omega1','\omega2','\omega3'})
            xlabel('Time [s]')
            ylabel('Angular Velocity, rad/s')
            title('Angular Velocity vs Time')
            
            subplot(3,1,3)
            hold on
            plot(sat.t,sat.q1,'color',c1,'linewidth',1.5)
            plot(sat.t,sat.q2,'color',c2,'linewidth',1.5)
            plot(sat.t,sat.q3,'color',c3,'linewidth',1.5)
            plot(sat.t,sat.q4,'-k','linewidth',1.5)
            hold off
            axis([0 tf 0 1],'auto y')
            grid on
            xlabel('Time [s]')
            ylabel('Quaternions')
            legend({'q1','q2','q3','q4'})
            title('Attitude Quaternions vs Time')
            
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
            
            figure
            plot(sat.t,sat.fuel,'k','linewidth',1)
            axis([0 tf 0 sat.fuel(1)])
            grid on
            xlabel('Time [s]')
            ylabel('Fuel, kg')
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
        
        %% Power analysis
        function pLoad = runPowerAnalysis(sat) 
            % Component power
            pADCSmin = 3.17;
            pADCSmax = 7.23;
            pPropmin = 0.25;
            pPropmax = 10.00;
            pCPU = 2*0.4;
            pCam = 0;
            pComm = 4;
            pGPS = 0.8;
            
            % Iteration
            for ii = 1:length(sat.t)             
                % Component power
                pADCS = pADCSmin + (pADCSmax-pADCSmin)*...
                        sum(abs([sat.T1(ii),sat.T2(ii),sat.T3(ii)]/sat.Tmax));
                pProp = pPropmin + (pPropmax-pPropmin)*...
                    sum(abs([sat.ux(ii),sat.uy(ii),sat.uz(ii)]/sat.umax));
                pLoad(ii) = pADCS+pProp+pCPU+pCam+pComm+pGPS+pGPS;
            end
        end
    end
end

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
