classdef ConvexSFFManeuver1 < handle
    
    properties
        X                       % State vector
        U                       % Control/decision vector
        T                       % Time vector
        Xq                      % States for quiver plot
        Uq                      % Controls for quiver plot
        optimalObjective        % Optimal cost
        slack
        
        samples                 % Number of samples between each timestep
        t0                      % Initial time
        dt                      % Time step size
        tf                      % Final time
        
        umax                    % Maximum control input magnitude
        umin                    % Minimum contorl input magnitude
        B                       % Continuous-time input matrix
        numInput                % Number of input/control states
        numState                % Number of states
        
        X0                      % Initial condition/state
        Xf                      % Terminal condition/state
        Nsim                    % Number of simulation steps
        
        motionModel             % Class of motion model used in optimization
        descriptor              % Name of the motion model used
    end
    
    methods
        function obj = ConvexSFFManeuver1(initStruct)
            obj.descriptor = initStruct.descriptor;
            fprintf(['Instantiating class ConvexSFFManeuver.m with model ' obj.descriptor '\n\n']);
            obj.motionModel = eval(strcat(obj.descriptor,'(initStruct)'));
            obj.t0 = initStruct.timeParams.t0;
            obj.dt = initStruct.timeParams.dt;
            obj.tf = initStruct.timeParams.tf;
            
            if isempty(initStruct.maneuverParams{1})
                obj.samples = [];
            else
                obj.samples = initStruct.maneuverParams{1};
            end
            obj.B = initStruct.maneuverParams{2};
            obj.numInput = size(obj.B,2);
            obj.numState = size(obj.B,1);
            obj.umax = initStruct.maneuverParams{3};
            obj.umin = initStruct.maneuverParams{4};
            obj.motionModel.makeTimeVector();
            obj.Nsim = length(obj.motionModel.time);
            obj.motionModel.makeDiscreteMatrices();
            obj.T = obj.motionModel.time;
            obj.X0 = obj.motionModel.initialCondition;
            obj.Xf = obj.motionModel.terminalCondition;
        end
        
        function obj = findFeasibleTime(obj,guess,increment)
            fprintf('\n');
            fprintf('Finding the minimum allowable time\n\n');
            diagnostics = 1;
            while diagnostics ~= 0
                nsim = guess;
                fprintf('Trying time-of-flight %f seconds\n\n',nsim*obj.dt);
                Bkk = [obj.motionModel.Bk,-obj.motionModel.Bk];
                u = sdpvar(2*obj.numInput,nsim);
                uslack = sdpvar(2*obj.numInput,nsim);
                x = sdpvar(obj.numState,nsim + 1);
                x1 = sdpvar(obj.numState,1);
                x2 = sdpvar(obj.numState,1);    
                constraints = [];
                constraints = [constraints, x(:,1) == obj.X0];
                for kk = 1:nsim
                    if strcmp(obj.descriptor,'HCW') == 1 
                        constraints = [constraints, x(:,kk+1) == ...
                            obj.motionModel.Ak*x(:,kk) + Bkk*u(:,kk)];
                    else
                        constraints = [constraints, ...
                            x(:,kk+1) == obj.motionModel.Ak(:,:,kk)*x(:,kk) + ...
                            Bkk(:,:,kk)*u(:,kk)];
                    end
                    for ii = 1:2*obj.numInput
                        constraints = [constraints, u(ii,kk) <= uslack(ii,kk)];
                        constraints = [constraints, -u(ii,kk) <= uslack(ii,kk)];
                        constraints = [constraints, 0 <= u(ii,kk) <= obj.umax];
                    end
                end
                constraints = [constraints, x(:,nsim+1) == obj.Xf];
                options = sdpsettings('solver','gurobi','verbose',2);
                parameters_in = {x1,x2};
                % We want to return the optimal inputs, the state history, and
                % the optimal value of the objective function, so we state that
                % here
                solutions_out = {u,x};
                % The controller object constructs the problem and transforms
                % it for use with whatever optimizer you choose (Gurobi!!!)
                controller = optimizer(constraints,[],options,parameters_in,...
                    solutions_out);
                % Solve the transfer problem using the boundry conditions
                [solutions,diagnostics] = controller{{obj.X0,obj.Xf}};
                if diagnostics ~= 0
                    guess = guess + increment;
                    fprintf('Infeasible time-of-flight, incrementing by %f seconds\n\n',increment*obj.dt);
                else
                    fprintf('Feasible time-of-flight found, Tf = %f seconds\n\n',nsim*obj.dt);
                    Uint = solutions{1};
                    obj.X = solutions{2};
%                     obj.optimalObjective = solutions{3};
                    if obj.numInput == 2
                        obj.U = [Uint(1,:)-Uint(3,:); Uint(2,:)-Uint(4,:)];
                    elseif obj.numInput == 3
                        obj.U = [Uint(1,:)-Uint(4,:); Uint(2,:)-Uint(5,:); Uint(3,:)-Uint(6,:)];
                    else
                    end
                    obj.tf = obj.dt*nsim;
                    obj.T = obj.t0:obj.dt:obj.tf;
                    obj.Nsim = nsim;
                    [obj.Xq,obj.Uq] = quivThrust(obj.T(1:end-1),transpose(obj.X),transpose(obj.U),obj.numInput,10,obj.descriptor);
                end
            end
        end
        
        function obj = minimizeFuel(obj)
            fprintf('\n');
            fprintf('Solving discrete-time, minimum fuel optimal control problem\n\n');
            Bkk = [obj.motionModel.Bk,-obj.motionModel.Bk];
            u = sdpvar(2*obj.numInput,obj.Nsim);
            uslack = sdpvar(2*obj.numInput,obj.Nsim);
            x = sdpvar(obj.numState,obj.Nsim + 1);
            x1 = sdpvar(obj.numState,1);
            x2 = sdpvar(obj.numState,1);
            constraints = [];
            objective = 0;
            constraints = [constraints, x(:,1) == obj.X0];
            tic
            for kk = 1:obj.Nsim
                for jj = 1:2*obj.numInput
                    objective = objective + abs(u(jj,kk));
                end
                if strcmp(obj.descriptor,'HCW') == 1 
                    constraints = [constraints, x(:,kk+1) == ...
                        obj.motionModel.Ak*x(:,kk) + Bkk*u(:,kk)];
                else
                    constraints = [constraints, ...
                        x(:,kk+1) == obj.motionModel.Ak(:,:,kk)*x(:,kk) + ...
                        Bkk(:,:,kk)*u(:,kk)];
                end
                for ii = 1:2*obj.numInput
%                     constraints = [constraints, uslack(ii,kk) <= obj.umax];
                    constraints = [constraints, u(ii,kk) <= obj.umax];
                    constraints = [constraints, -u(ii,kk) <= obj.umax];
                    constraints = [constraints, 0 <= u(ii,kk) <= obj.umax];
                end
%                 constraints = [constraints, sum(u(:,kk)) <= obj.umax];
            end
            constraints = [constraints, x(:,obj.Nsim+1) == obj.Xf];
            options = sdpsettings('solver','gurobi','verbose',2);
            parameters_in = {x1,x2};
            % We want to return the optimal inputs, the state history, and
            % the optimal value of the objective function, so we state that
            % here
            solutions_out = {u,x,objective,uslack};
            toc
            % The controller object constructs the problem and transforms
            % it for use with whatever optimizer you choose (Gurobi!!!)
            controller = optimizer(constraints,objective,options,parameters_in,...
                solutions_out);
            % Solve the transfer problem using the boundry conditions
            [solutions,~] = controller{{obj.X0,obj.Xf}};
            Uint = solutions{1};
            obj.X = solutions{2};
            obj.optimalObjective = solutions{3};
            if obj.numInput == 2
                obj.U = [Uint(1,:)-Uint(3,:); Uint(2,:)-Uint(4,:)];
            elseif obj.numInput == 3
                obj.U = [Uint(1,:)-Uint(4,:); Uint(2,:)-Uint(5,:); Uint(3,:)-Uint(6,:)];
            else
            end
            obj.slack = solutions{4};
            [obj.Xq,obj.Uq] = quivThrust(obj.T(1:end-1),transpose(obj.X),transpose(obj.U),obj.numInput,obj.dt,obj.descriptor);
        end
        
        function obj = minimizeEnergy(obj)
            fprintf('\n');
            fprintf('Solving discrete-time, minimum energy optimal control problem\n\n');
            u = sdpvar(obj.numInput,obj.Nsim);
            x = sdpvar(obj.numState,obj.Nsim + 1);
            x1 = sdpvar(obj.numState,1);
            x2 = sdpvar(obj.numState,1);
            constraints = [];
            objective = 0;
            constraints = [constraints, x(:,1) == obj.X0];
            for kk = 1:obj.Nsim
                for jj = 1:obj.numInput
                    objective = objective + u(jj,kk)^2;
                end
                if strcmp(obj.descriptor,'HCW') == 1
                    constraints = [constraints, x(:,kk+1) == ...
                        obj.motionModel.Ak*x(:,kk) + obj.motionModel.Bk*u(:,kk)];
                else
                    constraints = [constraints, ...
                        x(:,kk+1) == obj.motionModel.Ak(:,:,kk)*x(:,kk) + ...
                        obj.motionModel.Bk(:,:,kk)*u(:,kk)];
                end
            end
            constraints = [constraints, x(:,obj.Nsim+1) == obj.Xf];
            options = sdpsettings('solver','gurobi','verbose',2);
            parameters_in = {x1,x2};
            % We want to return the optimal inputs, the state history, and
            % the optimal value of the objective function, so we state that
            % here
            solutions_out = {u,x,objective};
            % The controller object constructs the problem and transforms
            % it for use with whatever optimizer you choose (Gurobi!!!)
            controller = optimizer(constraints,objective,options,parameters_in,...
                solutions_out);
            % Solve the transfer problem using the boundry conditions
            [solutions,~] = controller{{obj.X0,obj.Xf}};
            obj.U = solutions{1};
            obj.X = solutions{2};
            obj.optimalObjective = solutions{3};
            
            [obj.Xq,obj.Uq] = quivThrust(obj.T(1:end-1),transpose(obj.X),transpose(obj.U),obj.numInput,10,obj.descriptor);
        end
        
        function obj = minimizeEnergy_Constrained(obj)
            fprintf('\n');
            fprintf('Solving discrete-time, constrained, minimum energy optimal control problem\n\n');
            u = sdpvar(obj.numInput,obj.Nsim);
            x = sdpvar(obj.numState,obj.Nsim + 1);
            x1 = sdpvar(obj.numState,1);
            x2 = sdpvar(obj.numState,1);
            constraints = [];
            objective = 0;
            constraints = [constraints, x(:,1) == obj.X0];
            for kk = 1:obj.Nsim
                for jj = 1:obj.numInput
                    objective = objective + u(jj,kk)^2;
                end
                if strcmp(obj.descriptor,'HCW') == 1
                    constraints = [constraints, x(:,kk+1) == ...
                        obj.motionModel.Ak*x(:,kk) + obj.motionModel.Bk*u(:,kk)];
                else
                    constraints = [constraints, ...
                        x(:,kk+1) == obj.motionModel.Ak(:,:,kk)*x(:,kk) + ...
                        obj.motionModel.Bk(:,:,kk)*u(:,kk)];
                end
                for ii = 1:obj.numInput
                    constraints = [constraints, obj.umin <= u(ii,kk) <= obj.umax];
                end
            end
            constraints = [constraints, x(:,obj.Nsim+1) == obj.Xf];
            options = sdpsettings('solver','gurobi','verbose',2);
            parameters_in = {x1,x2};
            % We want to return the optimal inputs, the state history, and
            % the optimal value of the objective function, so we state that
            % here
            solutions_out = {u,x,objective};
            % The controller object constructs the problem and transforms
            % it for use with whatever optimizer you choose (Gurobi!!!)
            controller = optimizer(constraints,objective,options,parameters_in,...
                solutions_out);
            % Solve the transfer problem using the boundry conditions
            [solutions,~] = controller{{obj.X0,obj.Xf}};
            obj.U = solutions{1};
            obj.X = solutions{2};
            obj.optimalObjective = solutions{3};
            
            [obj.Xq,obj.Uq] = quivThrust(obj.T(1:end-1),transpose(obj.X),transpose(obj.U),obj.numInput,10,obj.descriptor);
        end
    end
end

function [Xout,Uout] = quivThrust(t,x,u,nu,step,id_flag)
if strcmp(id_flag,'HCW') == 1
    X = x(:,1:3);
elseif strcmp(id_flag,'LERM') == 1
    X = [x(:,1) x(:,3) x(:,5)];
elseif strcmp(id_flag,'GimAlfriendSTM') == 1
    X = [x(:,1) x(:,3) x(:,5)];
else
    error('Invalid Model Descriptor')
end
U = u(:,1:nu);
for ii = 1:length(t)
    if mod(ii,step) == 0
        Xout(ii,:) = X(ii,:);
        Uout(ii,:) = -2*U(ii,:);
    else
    end
end
end