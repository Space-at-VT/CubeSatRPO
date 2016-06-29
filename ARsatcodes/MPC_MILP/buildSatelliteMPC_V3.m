function [TrackingController,AttitudeController] = ...
    buildSatelliteMPC_V3(Nt,At,Bt,Ct,Qt,Rt,Umax,Shuttle,...
    maxTorque,Arot,Brot,Nr,Qr,Rr,timeHold,p,options)

if p == 1
    fprintf('Using 1-norm Convex-Modulated Model Predictive Control\n\n')
    [TrackingController] = ...
        BuildMPC_Trajectory_Tracker_OneNorm(Nt,At,Bt,Ct,Qt,Rt,Umax,Shuttle,timeHold,options);
elseif p == 2
    fprintf('Using Quadratic Convex-Modulated Model Predictive Control\n\n')
    [TrackingController] = ...
        BuildMPC_Trajectory_Tracker_Quadratic(Nt,At,Bt,Ct,Qt,Rt,Umax,Shuttle,timeHold,options);
elseif p == inf
    [TrackingController] = ...
        BuildMPC_Trajectory_Tracker_Inf(Nt,At,Bt,Ct,Qt,Rt,Umax,Shuttle,timeHold,options);
else
    error('Incorrect p-norm specified\n\n');
end

AttitudeController = buildMPC_Quat_Controller(maxTorque,Arot,Brot,Nr,Qr,Rr,options);

end

function [controller] = BuildMPC_Trajectory_Tracker_OneNorm(N,A,B,C,Q,R,Umax,Shuttle,timeHold,options)

nx = size(A,2);
nu = size(B,2);
ny = size(C,1);

%% Make obstacles (rectangular prisms)
Fuselage = Shuttle.shuttleStructV2.Fuselage.Dimensions;
Rudder = Shuttle.shuttleStructV2.Rudder.Dimensions;
RightWingMain = Shuttle.shuttleStructV2.RightWingMain.Dimensions;
RightWingStrake = Shuttle.shuttleStructV2.RightWingStrake.Dimensions;

Mbig = 1e7;
safetymargin = 0.2;

u = sdpvar(nu,N);
ur = sdpvar(nu,N);
x = sdpvar(nx,N+1);
r = sdpvar(ny,N+1);
e = sdpvar(nx,N+1);
[~,Sd,~] = dlqr(A,B,Q,R);
%% Define constraints and build optimizer
constraints = [];
objective = 0;
for kk = 1:N
    e(:,kk) = x(:,kk) - r(:,kk);
    ubar(:,kk) = u(:,kk) - ur(:,kk);
    objective = objective + norm(Q*(e(:,kk)),1) + norm(R*(ubar(:,kk)),1);
    constraints = [constraints, e(:,kk+1) == A*e(:,kk)+B*ubar(:,kk)];
    constraints = [constraints, 0 <= u(:,kk) <= Umax];
end
objective = objective + norm(Sd*(e(:,N+1)),1);
% objective = objective + norm((C*x(:,N+1) - r(:,N+1)),1);

[fuselageConstraints,xMINF,xMAxF] = buildPolytopicConstraints(N,Fuselage,Mbig,...
    x(1,:),x(2,:),x(3,:),safetymargin);
[rudderConstraints,xMINR,xMAxR] = buildPolytopicConstraints(N,Rudder,Mbig,x(1,:),...
    x(2,:),x(3,:),safetymargin);
[rightWingMainConstraints,xMINRW,xMAxRW] = buildPolytopicConstraints(N,RightWingMain,...
    Mbig,x(1,:),x(2,:),x(3,:),safetymargin/2);
[rightWingStrakeConstraints,xMINRS,xMAxRS] = buildPolytopicConstraints(N,...
    RightWingStrake,Mbig,x(1,:),x(2,:),x(3,:),safetymargin/2);

constraints = [constraints, fuselageConstraints, rudderConstraints,...
    rightWingMainConstraints, rightWingStrakeConstraints];

parameters_in = {x(:,1),[r],[ur]};
solutions_out = {[u],objective};
controller = optimizer(constraints, objective, options, parameters_in,...
    solutions_out);

end

function [controller] = BuildMPC_Trajectory_Tracker_Quadratic(N,A,B,C,Q,R,Umax,Shuttle,timeHold,options)

nx = size(A,2);
nu = size(B,2);
ny = size(C,1);

%% Make obstacles (rectangular prisms)
Fuselage = Shuttle.shuttleStructV2.Fuselage.Dimensions;
Rudder = Shuttle.shuttleStructV2.Rudder.Dimensions;
RightWingMain = Shuttle.shuttleStructV2.RightWingMain.Dimensions;
RightWingStrake = Shuttle.shuttleStructV2.RightWingStrake.Dimensions;

Mbig = 1e7;
safetymargin = 0.2;

u = sdpvar(nu,N);
ur = sdpvar(nu,N);
x = sdpvar(nx,N+1);
r = sdpvar(ny,N+1);
e = sdpvar(nx,N+1);
[~,Sd,~] = dlqr(A,B,Q,R);
%% Define constraints and build optimizer
constraints = [];
objective = 0;
for kk = 1:N
    e(:,kk) = x(:,kk) - r(:,kk);
    ubar(:,kk) = u(:,kk) - ur(:,kk);
    objective = objective + (e(:,kk))'*Q*(e(:,kk)) + (ubar(:,kk))'*R*(ubar(:,kk));
    constraints = [constraints, e(:,kk+1) == A*e(:,kk)+B*ubar(:,kk)];
    constraints = [constraints, 0 <= u(:,kk) <= Umax];
end

objective = objective + (e(:,N+1))'*Sd*(e(:,N+1));
% objective = objective + (C*x(:,N+1)-r(:,N+1))'*(C*x(:,N+1)-r(:,N+1));

[fuselageConstraints,xMINF,xMAxF] = buildPolytopicConstraints(N,Fuselage,Mbig,...
    x(1,:),x(2,:),x(3,:),safetymargin);
[rudderConstraints,xMINR,xMAxR] = buildPolytopicConstraints(N,Rudder,Mbig,x(1,:),...
    x(2,:),x(3,:),safetymargin);
[rightWingMainConstraints,xMINRW,xMAxRW] = buildPolytopicConstraints(N,RightWingMain,...
    Mbig,x(1,:),x(2,:),x(3,:),safetymargin/2);
[rightWingStrakeConstraints,xMINRS,xMAxRS] = buildPolytopicConstraints(N,...
    RightWingStrake,Mbig,x(1,:),x(2,:),x(3,:),safetymargin/2);

constraints = [constraints, fuselageConstraints, rudderConstraints,...
    rightWingMainConstraints, rightWingStrakeConstraints];

parameters_in = {x(:,1), [r],[ur]};
solutions_out = {[u],objective};
controller = optimizer(constraints, objective, options, parameters_in,...
    solutions_out);

end

function [controller] = BuildMPC_Trajectory_Tracker_Inf(N,A,B,C,Q,R,Umax,Shuttle,timeHold,options)

nx = size(A,2);
nu = size(B,2);
ny = size(C,1);

%% Make obstacles (rectangular prisms)
Fuselage = Shuttle.shuttleStructV2.Fuselage.Dimensions;
Rudder = Shuttle.shuttleStructV2.Rudder.Dimensions;
RightWingMain = Shuttle.shuttleStructV2.RightWingMain.Dimensions;
RightWingStrake = Shuttle.shuttleStructV2.RightWingStrake.Dimensions;

Mbig = 1e7;
safetymargin = 0.2;

u = sdpvar(nu,N);
x = sdpvar(nx,N+1);
r = sdpvar(ny,N+1);

options = sdpsettings('solver','gurobi','verbose',0);
[~,Sd,~] = dlqr(A,B,Q,R);
%% Define constraints and build optimizer
constraints = [];
objective = 0;
for kk = 1:N
    objective = objective + norm(Q*(C*x(:,kk)-r(:,kk)),inf) + norm(R*u(:,kk),inf);
    constraints = [constraints, x(:,kk+1) == A*x(:,kk)+B*u(:,kk)];
    constraints = [constraints, 0 <= u(:,kk) <= Umax];
end
objective = objective + norm(Sd*(C*x(:,N+1) - r(:,N+1)),inf);
% objective = objective + norm((C*x(:,N+1) - r(:,N+1)),inf);

[fuselageConstraints,xMINF,xMAxF] = buildPolytopicConstraints(N,Fuselage,Mbig,...
    x(1,:),x(2,:),x(3,:),safetymargin);
[rudderConstraints,xMINR,xMAxR] = buildPolytopicConstraints(N,Rudder,Mbig,x(1,:),...
    x(2,:),x(3,:),safetymargin);
[rightWingMainConstraints,xMINRW,xMAxRW] = buildPolytopicConstraints(N,RightWingMain,...
    Mbig,x(1,:),x(2,:),x(3,:),safetymargin/2);
[rightWingStrakeConstraints,xMINRS,xMAxRS] = buildPolytopicConstraints(N,...
    RightWingStrake,Mbig,x(1,:),x(2,:),x(3,:),safetymargin/2);

constraints = [constraints, fuselageConstraints, rudderConstraints,...
    rightWingMainConstraints, rightWingStrakeConstraints];

parameters_in = {x(:,1), [r]};
solutions_out = {[u],objective};
controller = optimizer(constraints, objective, options, parameters_in,...
    solutions_out);

end

function controller = buildMPC_Quat_Controller(maxTorque,A,B,N,Q,R,options)
q = sdpvar(3,N+1);
w = sdpvar(3,N+1);

x = [q; w];

tau = sdpvar(3,N);
[~,P,~] = dlqr(A,B,Q,R);

constraints = [];
objective = 0;

for kk = 1:N
    %     objective = objective + norm(Q*x(:,kk),px) + norm(R*tau(:,kk),pu);
    objective = objective + x(:,kk)'*Q*x(:,kk) + tau(:,kk)'*R*tau(:,kk);
    constraints = [constraints, x(:,kk+1) == A*x(:,kk) + B*tau(:,kk)];
    constraints = [constraints, -maxTorque <= tau(:,kk) <= maxTorque];
end
cosntraints = [constraints, x(:,N+1)'*P*x(:,N+1) <= 0.1];
objective = objective + x(:,N+1)'*P*x(:,N+1);

parameters_in = {x(:,1)};
solutions_out = {tau(:,1),objective};
controller = optimizer(constraints, objective, options, parameters_in, ...
    solutions_out);
end