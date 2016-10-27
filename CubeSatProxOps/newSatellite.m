%% New Satellite
% Translational trajcetory generation is all the same as the summer.
% Organized to be more easily commanded. All attitude modeling is still in
% development. Basic damping and PD in included for perturbations caused
% from the off-centering of thrusters. As of 9/8 the model is not coupled
% with translational movement. The linearity of the model may limit the
% accuracy of coupling rotation with translation.

classdef newSatellite < handle
    %% Default properties of a basic 6u cubesat
    properties
        %% Graphics
        name = 'CubeSat'            %Satellite name
        EOM = 'LERM'                 %Relative motion model
        mode = 'approach'           %Satellite objective
        color = 'b'                 %Graph color
        
        %% Satellite parameters
        umax = 0.25                 %Thrust,                N
        ISP = 150                   %Specific impulse,      s
        dryMass = 13                %Dry mass,              kg
        fuel = 0.5                  %Fuel mass,             kg
        vmax = 0.5                  %Max velocity,          m/s
        bnd = [0.1,0.3,0.2]         %Satellite size,        m
        Tmax = 0.25                 %Max reaction torque
        kp = 0.1                    %Position damping
        kd = 0.7                    %Velocity damping
        ki = 0.3                    %Integral damping
        d = [0,0,0]        %Thruster misalignment
        % moment arm,           m
        
        %% Trajectory
        t = 0
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
        wb2 = 0                     %over time,             rad/s
        wb3 = 0                     %                       rad/s
        q1 = 0                      %Quaternions
        q2 = 0
        q3 = 0
        q4 = 1;                     %Scalar
        
        point = 0                   %Attitude pointing y/n, binary
        pt = [0,0,0]                %Attitude point target, m
        
        %% Debug
        u
        a
        J = [];
        flag = []                   %Exit flag
        makeMovie = 0;              %Save movie struct
        vid
        frames
        
    end
    properties (Dependent)
        m                           %Total mass,            kg
        mdot                        %Mass flow rate,        kg/s
        p                           %Position vector,       m
        v                           %Velocity vector,       m/s
        w                           %Angular velocity,      rad/s
        qb                          %Quaternions vector,
        Rib                         %Rotation matrix b2i
        Rbi                         %Rotation matrix i2b
        ubnd                        %Upper bound,           m
        lbnd                        %Lower bound,           m
        I                           %Moments of Inertia,    kg/m^2
    end
    methods
        % Total mass (Dependent)
        function m = get.m(obj)
            m = obj.fuel+obj.dryMass;
        end
        % Mass flow rate (Dependent)
        function mdot = get.mdot(obj)
            mdot = 1/obj.ISP/9.81;
        end
        % Current position vector
        function p = get.p(obj)
            p = [obj.x(end),obj.y(end),obj.z(end)];
        end
        % Current velocity vector
        function v = get.v(obj)
            v = [obj.vx(end),obj.vy(end),obj.vz(end)];
        end
        % Current thrust vector
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
            qx = [0 -q(3) q(2)
                q(3) 0 -q(1)
                -q(2) q(1) 0];
            Rbi = (q4^2-q'*q)*eye(3)+2*(q*q')-2*q4*qx;
        end
        % Current body to inertial rotation matrix
        function Rib = get.Rib(obj)
            Rib = inv(obj.Rbi);
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
        % Video making functions
        function sat = addFrame(sat,fig)
            iter = length(sat.frames);
            sat.frames = getframe(fig);
            
        end
        function sat = renderFrames(sat)
            sat.vid = VideoWriter(sat.name);
            open(sat.vid);
            for ii = 1:length(sat.frames)
                writeVideo(sat.vid,sat.frames(ii))
            end
            close(sat.vid);
        end
        
        %% Basic point to point movement with collision avoidance
        function sat = approach(sat,scenario,p,lbnd,ubnd)
            if nargin < 4 || isempty(lbnd)
                lbnd = [];
                ubnd = [];
            end
            if sat.fuel > 0
                w1 = 5e-2; %Thrust
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
                
                iter = length(sat.x);
                options = optimoptions(@intlinprog,'Display','None','MaxTime',1);
                [U,fval,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub,options);
                
                if isempty(U) == 1
                    sat.u = sat.u(7:end);
                    sat.a = sat.a(4:end);
                    sat.J(iter) = sat.J(iter-1);
                    sat.flag(iter) = 3;
                else
                    sat.u = U(1:Nvar);
                    sat.a = U(Nvar+1:Nvar+Neom);
                    sat.J(iter) = fval;
                    sat.flag(iter) = exitflag;
                end
                sat.signalsProp(scenario);
            else
                sat.driftProp(scenario);                
            end
            sat.attitudeProp(scenario);
            sat.t(iter+1) = sat.t(iter)+scenario.dt;
        end
        
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
                %
                %                 [A1,b1] = holdProximity([],[],sat,scenario,lbnd,ubnd);
                %                 [A2,b2] = holdProximitytest([],[],sat,scenario,lbnd,ubnd);
                
                iter = length(sat.x);
                options = optimoptions(@intlinprog,'Display','None','MaxTime',1);%
                [U,fval,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub,options);
                
                if isempty(U) == 1
                    sat.u = sat.u(7:end);
                    sat.a = sat.a(4:end);
                    sat.J(iter) = sat.J(iter-1);
                    sat.flag(iter) = 3;
                else
                    sat.u = U(1:Nvar);
                    sat.a = U(Nvar+1:end);
                    sat.J(iter) = fval;
                    sat.flag(iter) = exitflag;
                end
                
                sat.signalsProp(scenario);
            else
                sat.driftProp(scenario);
            end
            sat.t(iter+1) = sat.t(iter)+scenario.dt;
            sat.attitudeProp(scenario);
        end
        
        %% Propagate with no control for given time
        function sat = propagate(sat,scenario,tspan)
            for ii = 1:scenario.dt:tspan
                iter = length(sat.x);
                sat.J(iter) = 0;
                sat.driftProp(scenario);
                sat.attitudeProp(scenario);
                sat.t(iter+1) = sat.t(iter)+scenario.dt;
            end
        end
        
        %% Non-MPC Long-Range Maneuver (not real time)
        function sat = phaseManeuver(sat,scenario,Xf,tf,dt)
            Told = scenario.T;
            dtold = scenario.dt;
            scenario.T = tf;
            scenario.dt = dt;
            
            Nvar = scenario.Nvar;
            Neom = scenario.Neom;
            
            % Function coefficients
            f = [dt*ones(Nvar,1); %Control thrusts
                zeros(Neom,1);
                zeros(3,1)];
            
            % Parameter bounds, lower & upper
            lb = [zeros(Nvar,1);   %Control thrusts
                -inf*ones(Neom,1);
                zeros(3,1)];
            
            ub = [ones(Nvar,1);   %Control thrusts
                inf*ones(Neom,1);
                ones(3,1)];
            
            % Equality contraints
            Aeq = []; beq = [];
            [Aeq,beq] = setEOM(Aeq,beq,sat,scenario);
            [Aeq,beq] = setPhaseState(Aeq,beq,sat,scenario,Xf);
            
            % Inequality contraints
            A = [];   b = [];
            
            % Integer constraint
            intcon = [];
            
            % options = optimoptions(@linprog);
            [U,fval,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub);
            %             U = Aeq\beq;
            
            jj = 1;
            for ii = 1:6:Nvar
                iter = length(sat.x);
                sat.J(iter) = fval;
                sat.flag(iter) = exitflag;
                
                sat.u = U(ii:ii+5);
                sat.a = U(Nvar+jj:Nvar+jj+2);
                
                sat = signalsProp(sat,scenario);
                sat = attitudeProp(sat,scenario);
                
                sat.t(iter+1) = sat.t(iter)+scenario.dt;
                jj = jj+3;
            end
            
            scenario.T = Told;
            scenario.dt = dtold;
            
        end
        
        %% Convert solver signals to propagation
        function sat = signalsProp(sat,scenario)
            u = sat.u;
            iter = length(sat.x);
            
            % Control signals
            ub = ([u(1)-u(2)
                   u(3)-u(4)
                   u(5)-u(6)]);
            ui = sat.Rib*ub;
            
            sat.ub1(iter) = sat.umax*ub(1);
            sat.ub2(iter) = sat.umax*ub(2);
            sat.ub3(iter) = sat.umax*ub(3);
            
            sat.ux(iter) = sat.umax*ui(1);
            sat.uy(iter) = sat.umax*ui(2);
            sat.uz(iter) = sat.umax*ui(3);
            
            ax = sat.a(1);
            ay = sat.a(2);
            az = sat.a(3);
            
            % New velocity
            dt = scenario.dt;
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
            sat.fuel = sat.fuel-sum(u(1:6))*sat.umax*sat.mdot;
        end
        
        %% Propagate trajectory with no fuel
        function sat = driftProp(sat,scenario)
            dt = scenario.dt;
            
            % Switch equations of motion model state matrix
            switch sat.EOM
                case 'HCW'
                    A = HCW(scenario);
                case 'LERM'
                    A = LERM(scenario,sat.t(end));
            end
            
            % Calculate new state
            iter = length(sat.x);
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
            
            % New velocity
            sat.vx(iter+1) = sat.vx(iter)+DX(4)*dt;
            sat.vy(iter+1) = sat.vy(iter)+DX(5)*dt;
            sat.vz(iter+1) = sat.vz(iter)+DX(6)*dt;
            
            % New position
            sat.x(iter+1) = sat.x(iter)+sat.vx(iter)*dt;
            sat.y(iter+1) = sat.y(iter)+sat.vy(iter)*dt;
            sat.z(iter+1) = sat.z(iter)+sat.vz(iter)*dt;
            
            % Update control vector
            sat.J(iter+1) = sat.J(iter);
            sat.ux(iter+1) = 0;
            sat.uy(iter+1) = 0;
            sat.uz(iter+1) = 0;
            
            sat.ub1(iter+1) = 0;
            sat.ub2(iter+1) = 0;
            sat.ub3(iter+1) = 0;
            
            % Debug flag
            sat.flag(iter) = 3;
        end
        
        %% Propagate attitude
        function sat = attitudeProp(sat,scenario)
            % Reaction wheel torques
            iter = length(sat.q1);
            dt = scenario.dt;
            
            % Still in progress - use PD contorolled reaction wheel torques to point
            % satellite at user-defined point.
            % Else the satellite just minimizes divergence from the inertial axes.
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
                
                sat.T1(iter) = -sat.kp*(qe(1)*qe(4))-sat.kd*sat.wb1(iter);
                sat.T2(iter) = -sat.kp*(qe(2)*qe(4))-sat.kd*sat.wb2(iter);
                sat.T3(iter) = -sat.kp*(qe(3)*qe(4))-sat.kd*sat.wb3(iter);
                sat.T1(iter+1) = 0;
                sat.T2(iter+1) = 0;
                sat.T3(iter+1) = 0;
            else
                sat.T1(iter) = -sat.kp*(sat.q1(iter))-sat.kd*sat.wb1(iter);
                sat.T2(iter) = -sat.kp*(sat.q2(iter))-sat.kd*sat.wb2(iter);
                sat.T3(iter) = -sat.kp*(sat.q3(iter))-sat.kd*sat.wb3(iter);
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
                -q(2) q(1) 0];
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
        
        %% Plot relative trajectory
        function plotTrajectory(sat,lbnd,ubnd,unit)
            if nargin < 2 || isempty(unit),unit = 5;end
            fig = figure(1);
            
            % Plot obstacle bounds
            for ii = 1:size(lbnd,1)
                plotObstacle(lbnd(ii,:),ubnd(ii,:),'-k');
            end
            
            % Plot satellite trajectory
            hold on
            for jj = 1:length(sat)
                p1style = strcat('-',sat.color);
                p4style = 'ks';
                plot3(sat.y,sat.z,sat.x,p1style,'linewidth',1.5);
%                 quiver3(sat.y,sat.z,sat.x,-sat.uy,-sat.uz,-sat.ux,1,'r','linewidth',1.5);
                plot3(sat.p(2),sat.p(3),sat.p(1),p4style,'linewidth',2,'markersize',10);
                
                % Plot satellite body axes
                R = unit*sat.Rib;
                plot3([sat.p(2),sat.p(2)+R(2,1)'],[sat.p(3),sat.p(3)+R(3,1)'],...
                    [sat.p(1),sat.p(1)+R(1,1)'],'b','linewidth',1.5);
                plot3([sat.p(2),sat.p(2)+R(2,2)'],[sat.p(3),sat.p(3)+R(3,2)'],...
                    [sat.p(1),sat.p(1)+R(1,2)'],'r','linewidth',1.5);
                plot3([sat.p(2),sat.p(2)+R(2,3)'],[sat.p(3),sat.p(3)+R(3,3)'],...
                    [sat.p(1),sat.p(1)+R(1,3)'],'g','linewidth',1.5);
            end
            hold off
            
            % Axis labels
            grid on
            zlabel('Radial, x [m]')
            xlabel('In-track, y [m]')
            ylabel('Cross-track, z [m]')
            title('Relative Trajectory')
            axis('tight','equal','vis3d')
            camva(10)
            view(145,15)
            
            drawnow
        end
        
        %% Plot control signals and state over time
        function plotControls(sat)
            t = sat.t;
            tf = sat.t(end);
            
            %
            figure
            subplot(3,3,1)
            hold on
            stairs(t,sat.ub1/sat.umax,'-b','linewidth',2)
            plot([0 tf],[0 0],'--k','linewidth',2)
            axis([0 tf -1.5 1.5])
            grid on
            title('Control Signals vs Time')
            ylabel('ub1')
            
            subplot(3,3,4)
            hold on
            stairs(t,sat.ub2/sat.umax,'-r','linewidth',2)
            plot([0 tf],[0 0],'--k','linewidth',2)
            axis([0 tf -1.5 1.5])
            grid on
            ylabel('ub2')
            
            subplot(3,3,7)
            hold on
            stairs(t,sat.ub3/sat.umax,'-g','linewidth',2)
            plot([0 tf],[0 0],'--k','linewidth',2)
            axis([0 tf -1.5 1.5])
            grid on
            xlabel('Time [s]')
            ylabel('ub3')
            
            % Velocity
            subplot(3,3,2)
            plot(t,sat.vx,'-b','linewidth',2)
            axis([0 tf 0 1],'auto y')
            grid on
            ylabel('Vx [m/s]')
            title('Velocity vs Time')
            
            subplot(3,3,5)
            plot(t,sat.vy,'-r','linewidth',2)
            axis([0 tf 0 1],'auto y')
            grid on
            ylabel('Vy [m/s]')
            
            subplot(3,3,8)
            plot(t,sat.vz,'-g','linewidth',2)
            axis([0 tf 0 1],'auto y')
            grid on
            xlabel('Time [s]')
            ylabel('Vz [m/s]')
            
            % Position
            subplot(3,3,3)
            plot(t,sat.x,'-b','linewidth',2)
            axis([0 tf 0 1],'auto y')
            grid on
            ylabel('x [m]')
            title('Position vs Time')
            
            subplot(3,3,6)
            plot(t,sat.y,'-r','linewidth',2)
            axis([0 tf 0 1],'auto y')
            grid on
            ylabel('y [m]')
            
            subplot(3,3,9)
            plot(t,sat.z,'-g','linewidth',2)
            axis([0 tf 0 1],'auto y')
            grid on
            xlabel('Time [s]')
            ylabel('z [m]')
            
            %
            figure
            subplot(3,1,1)
            hold on
            plot(t,sat.T1,'-b','linewidth',2)
            plot(t,sat.T2,'-r','linewidth',2)
            plot(t,sat.T3,'-g','linewidth',2)
            hold off
            axis([0 tf 0 1],'auto y')
            grid on
            legend({'T1','T2','T3'})
            xlabel('Time [s]')
            ylabel('Reaction Torques, Nm')
            title('Reaction Torques vs Time')
            
            subplot(3,1,2)
            hold on
            plot(t,sat.wb1,'-b','linewidth',2)
            plot(t,sat.wb2,'-r','linewidth',2)
            plot(t,sat.wb3,'-g','linewidth',2)
            hold off
            axis([0 tf 0 1],'auto y')
            grid on
            legend({'\omega1','\omega2','\omega3'})
            xlabel('Time [s]')
            ylabel('Angular Velocity, rad/s')
            title('Angular Velocity vs Time')
            
            subplot(3,1,3)
            hold on
            plot(t,sat.q1,'-b','linewidth',2)
            plot(t,sat.q2,'-r','linewidth',2)
            plot(t,sat.q3,'-g','linewidth',2)
            plot(t,sat.q4,'-k','linewidth',2)
            hold off
            axis([0 tf 0 1],'auto y')
            grid on
            xlabel('Time [s]')
            ylabel('Quaternions')
            legend({'q1','q2','q3','q4'})
            title('Attitude Quaternions vs Time')
            
            %
            figure
            subplot(4,1,1)
            hold on
            stairs(t,sat.ub1/sat.umax,'-k','linewidth',1)
            plot([0 tf],[0 0],'--k','linewidth',1)
            axis([0 tf -1.5 1.5])
            grid on
            title('Control Signals vs Time')
            xlabel('Time [s]')
            ylabel('u_{b1}')
            
            subplot(4,1,2)
            hold on
            stairs(t,sat.ub2/sat.umax,'-k','linewidth',1)
            plot([0 tf],[0 0],'--k','linewidth',1)
            axis([0 tf -1.5 1.5])
            grid on
            xlabel('Time [s]')
            ylabel('u_{b2}')
            
            subplot(4,1,3)
            hold on
            stairs(t,sat.ub3/sat.umax,'-k','linewidth',1)
            plot([0 tf],[0 0],'--k','linewidth',1)
            axis([0 tf -1.5 1.5])
            grid on
            xlabel('Time [s]')
            ylabel('u_{b3}')
            
            subplot(4,1,4)
            plot(t,sat.J,'k','linewidth',1)
            axis([0 tf 0 1],'auto y')
            grid on
            xlabel('Time [s]')
            ylabel('Cost Function, J')
            
        end
    end
end