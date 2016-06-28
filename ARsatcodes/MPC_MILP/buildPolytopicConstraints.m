function [constraints,XMIN,XMAX] = buildPolytopicConstraints(Nsim,XOBS,Mbig,x,y,z,...
    safetyMargin)
constraints = [];
XMIN = [XOBS(1) - safetyMargin; XOBS(2) - safetyMargin; XOBS(3) - ...
    safetyMargin];
XMAX = [XOBS(4) + safetyMargin; XOBS(5) + safetyMargin; XOBS(6) + ...
    safetyMargin];

oxmin = binvar(1,Nsim);
oymin = binvar(1,Nsim);
ozmin = binvar(1,Nsim);
% size(oxmin)

oxmax = binvar(1,Nsim);
oymax = binvar(1,Nsim);
ozmax = binvar(1,Nsim);
for kk = 1:Nsim
    constraints = [constraints, x(kk) <= XMIN(1) + Mbig*oxmin(kk)];
    constraints = [constraints, y(kk) <= XMIN(2) + Mbig*oymin(kk)];
    constraints = [constraints, z(kk) <= XMIN(3) + Mbig*ozmin(kk)];
    constraints = [constraints, -x(kk) <= -XMAX(1) + Mbig*oxmax(kk)];
    constraints = [constraints, -y(kk) <= -XMAX(2) + Mbig*oymax(kk)];
    constraints = [constraints, -z(kk) <= -XMAX(3) + Mbig*ozmax(kk)];
    constraints = [constraints, (oxmin(kk) + oymin(kk) + ozmin(kk) + ...
        oxmax(kk) + oymax(kk) + ozmax(kk)) <= 5];
end
end