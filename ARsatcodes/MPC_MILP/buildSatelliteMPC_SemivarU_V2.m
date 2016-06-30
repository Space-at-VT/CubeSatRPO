function [TrackingController,AttitudeController] = ...
    buildSatelliteMPC_SemivarU_V2(Nt,At,Bt,Ct,Qt,Rt,Umax,Umin,Shuttle,...
    maxTorque,Arot,Brot,Nr,Qr,Rr,timeHold,p,options)

if p == 1
[TrackingController] = ...
    BuildMPC_Trajectory_Tracker_OneNorm_SemivarU(Nt,At,Bt,Ct,Qt,Rt,Umax,Umin,Shuttle,options);
elseif p == 2
[TrackingController] = ...
    BuildMPC_Trajectory_Tracker_Quadratic_SemivarU(Nt,At,Bt,Ct,Qt,Rt,Umax,Umin,Shuttle,options);
else
    error('Incorrect p-norm specified\n\n');
end

[AttitudeController] = buildMPC_Quat_Controller(maxTorque,Arot,Brot,Nr,Qr,Rr,options);

end

function [controller] = BuildMPC_Trajectory_Tracker_OneNorm_SemivarU(N,A,B,C,Q,R,Umax,Umin,Shuttle,options)
fprintf('Using 1-norm Mixed-Integer Model Predictive Control\n\n')
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
    
u = semivar(nu,N);
ur = sdpvar(nu,N);
x = sdpvar(nx,N+1);
r = sdpvar(ny,N+1);

[~,Sd,~] = dlqr(A,B,Q,R);
%% Define constraints and build optimizer
constraints = [];
objective = 0;
for kk = 1:N
    objective = objective + norm(Q*(C*x(:,kk)-r(:,kk)),1) + norm(R*(u(:,kk)-ur(:,kk)),1);
    constraints = [constraints, x(:,kk+1) == A*x(:,kk)+B*u(:,kk)];
    constraints = [constraints, Umin <= u(:,kk) <= Umax];
end
objective = objective + norm(Sd*(C*x(:,N+1) - r(:,N+1)),1);
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

function [controller] = BuildMPC_Trajectory_Tracker_Quadratic_SemivarU(N,A,B,C,Q,R,Umax,Umin,Shuttle,options)
fprintf('Using Quadratic Mixed-Integer Model Predictive Control\n\n')
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
    
u = semivar(nu,N);
ur = sdpvar(nu,N);
x = sdpvar(nx,N+1);
r = sdpvar(ny,N+1);

[~,Sd,~] = dlqr(A,B,Q,R);
%% Define constraints and build optimizer
constraints = [];
objective = 0;
for kk = 1:N
    objective = objective + (C*x(:,kk)-r(:,kk))'*Q*(C*x(:,kk)-r(:,kk)) + (u(:,kk)-ur(:,kk))'*R*(u(:,kk)-ur(:,kk));
    constraints = [constraints, x(:,kk+1) == A*x(:,kk)+B*u(:,kk)];
    constraints = [constraints, Umin <= u(:,kk) <= Umax];
end

objective = objective + (C*x(:,N+1)-r(:,N+1))'*Sd*(C*x(:,N+1)-r(:,N+1));

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

function [controller] = buildMPC_Quat_Controller(maxTorque,A,B,N,Q,R,options)
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
constraints = [constraints, x(:,N+1)'*P*x(:,N+1) <= 0.1];
objective = objective + x(:,N+1)'*P*x(:,N+1);

parameters_in = {x(:,1)};
solutions_out = {tau(:,1),objective};
[controller] = optimizer(constraints, objective, options, parameters_in, ...
    solutions_out);
end