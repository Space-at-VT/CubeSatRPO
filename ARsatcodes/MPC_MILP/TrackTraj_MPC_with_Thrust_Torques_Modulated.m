function [X,err,yaw,pitch,roll,yawdot,pitchdot,rolldot,Q,Thruster,Torques,deltaVTracking,mTrack,...
    Xqt,Uqt,rotEnergy,tranEnergy,trajDiag,attDiag,trajObj,attObj,punt,pctPunt] = TrackTraj_MPC_with_Thrust_Torques_Modulated(...
    Time,d,Nt,ref,TrackingController,AttitudeController,nx,nu,ny,ntau,...
    Nsim,Isp,g0,m0,Ts,Imat,a,ecc,mu,n,rho,Umin)

X = zeros(nx,Nsim);
Thruster = zeros(3,Nsim);
Torques = zeros(ntau,Nsim);
Ref = zeros(ny,Nsim);
XN = zeros(13,Nsim);
yaw = zeros(1,Nsim);
pitch = yaw;
roll = yaw;
yawdot = yaw;
pitchdot = yaw;
rolldot = yaw;
xt = d(7:12);
punt = 0;
for ii = 1:length(Time)
    future_r = ref(1:ny,ii:ii+Nt);
    
    % Compute optimal control
    [tsolutions,trajDiag(ii)] = TrackingController{{xt(:),future_r}};
    U = tsolutions{1};
    trajObj(ii) = tsolutions{2};
    xattin = d(1:6); xattin = xattin(:);
    [rsolutions,attDiag(ii)] = AttitudeController{{xattin}};
    Tau = rsolutions{1};
    attObj(ii) = rsolutions{2};
    Torques(:,ii) = Tau;
    for jj = 1:length(U)
        if abs(U(jj)) <= Umin
            U(jj) = 0;
        else
        end
    end
    d = [sqrt(1 - d(1)^2 - d(2)^2 - d(3)^2); d(:)];
    [~,xd] = ode15s(@coupledMPC_torques,ii:ii+1,d,[],mu,n,m0,Imat,Tau,U(:,1),a,ecc,rho);
    Thruster(:,ii) = [U(1) - U(4); U(2) - U(5); U(3) - U(6)];
    d = xd(end,:);
    d(1:4) = quatRenorm(d(1:4));
    Q(:,ii) = d(1:4);
    XN(:,ii) = d';
    xt(1:3) = d(8:10)';
    xt(4:6) = d(11:13);
    X(:,ii) = xt(:,1);
    Ref(:,ii) = future_r(:,1);
    err(:,ii) = X(:,ii) - Ref(:,ii);
    ypr = EP2Euler321(XN(1:4,ii));
    yaw(ii) = ypr(1)*180/pi;
    pitch(ii) = ypr(2)*180/pi;
    roll(ii) = ypr(3)*180/pi;
    yprDot = BmatEuler321(ypr)*XN(5:7,ii);
    yawdot(ii) = yprDot(1)*180/pi;
    pitchdot(ii) = yprDot(2)*180/pi;
    rolldot(ii) = yprDot(3)*180/pi;
    
    rotIP(ii) = dot(Tau,Tau);
    tranIP(ii) = dot([U(1) - U(4); U(2) - U(5); U(3) - U(6)],[U(1) - U(4); U(2) - U(5); U(3) - U(6)]);
    if mod(ii,250) == 0
        disp(ii)
    else
    end
    if trajDiag(ii) ~= 0
        punt = punt + 1;
    else
    end
    d = d(2:end);
end

YPR = [yaw; pitch; roll];
YPRDOT = [yawdot; pitchdot; rolldot];
[~,deltaVTracking,mTrack] = ...
    TotalFuel(Time,Thruster,m0,Isp,g0,Ts);
[Xqt,Uqt] = quivThrust(Time,X',Thruster,2);

rotEnergy = trapz(Time,rotIP);
tranEnergy = trapz(Time,tranIP);
pctPunt = (punt/length(Time))*100;
fprintf('\n');
fprintf(['Punted ' num2str(punt) ' times, or ' num2str(pctPunt) ' percent of time\n']);
end

function [fuelCost,dv,mt] = TotalFuel(Time,U,m0,Isp,g0,dt)

fuelCost = 0;
for ii = 1:length(Time)
    fuelCost = fuelCost + norm(U(:,1),1);
end
mt = zeros(1,length(Time));

mt(1) = m0;
for ii = 1:length(Time)-1
    ukn(ii) = norm(U(:,ii));
    mt(ii+1) = mt(ii) + dt*(-ukn(ii)/(Isp*g0));
end
dv = deltaV(m0,Isp,g0,mt(end));
end

function [Xout,Uout] = quivThrust(t,x,u,step)

X = x(:,1:3); U = u(1:3,:)';
for ii = 1:length(U)
    if mod(ii,step) == 0
        Xout(ii,:) = X(ii,:);
        Uout(ii,:) = -U(ii,:);
    else
    end
end
end

function DX = coupledMPC_torques(t,X,mu,n,mass,Imat,tau,force,a,ecc,rho)

if isempty(tau)
    tau = zeros(3,1);
else
end

if isempty(force)
    force = zeros(3,1);
else
end

r1x = rho(1);
r1z = rho(2);
r2x = rho(3);
r2y = rho(4);
r3y = rho(5);
r3z = rho(6);
r4y = rho(7);
r4z = rho(8);
r5x = rho(9);
r5y = rho(10);
r6x = rho(11);
r6z = rho(12);

q = X(1:4);
q = quatRenorm(q);
wbob = X(5:7);
Rbo = EP2C(q);

x = X(8); y = X(9); z = X(10);
xd = X(11); yd = X(12); zd = X(13);

% Time varying coefficients and solution to Kepler's equation
M = n.*t;
M = M(:);
[EccAnom,~] = kepler(M,ecc);
f = 2.*atan(sqrt((1+ecc)./(1-ecc)).*tan(EccAnom./2));
r = (a.*(1-ecc.^2))./(1+ecc.*cos(f));
fdot = sqrt(mu.*a.*(1-ecc.^2)).*(1+ecc.*cos(f)).^2./(a.^2.*(1-ecc.^2).^2);
rdot = ecc.*sin(f).*sqrt(mu.*a.*(1-ecc.^2))./(a.*(1-ecc.^2));
fddot = -2.*rdot.*fdot./r;

gamma = ((r+x).^2+y.^2+z.^2).^(1/2);

Fvec = [xd;
    yd;
    zd;
    2.*fdot.*yd + fddot.*y + fdot.^2.*x + mu./r.^2 - mu.*(x+r)./gamma.^3;
    -2.*fdot.*xd - fddot.*x + fdot.^2.*y - mu.*y./gamma.^3;
    -mu.*z./gamma.^3];

Bmat = [zeros(3,6); Rbo -Rbo];

woio = [0 0 -n]';
wbib = wbob + Rbo*woio;
RboDot = -skewMat(wbob)*Rbo;
c1 = Rbo(:,1);

Bq = BmatEP(q);
tau_gg = 3*n^2*skewMat(c1)*Imat*c1;

f_inert_body = Imat^(-1)*(-skewMat(wbib)*(Imat*wbib));
f_tau_gg_body = Imat^(-1)*tau_gg;
f_tau_input_body = Imat^(-1)*tau;

f_add = RboDot*woio;

UP = Rbo*force(1:3);
UM = -Rbo*force(4:6);

upx = UP(1);
upy = UP(2);
upz = UP(3);
umx = UM(1);
umy = UM(2);
umz = UM(3);


N1 = Imat^(-1)*[r1z*umy, 0, -r1x*umy]';
N2 = Imat^(-1)*[r2y*upz, -r2x*upz, 0]';
N3 = Imat^(-1)*[0, -r3z*umx, r3y*umx]';
N4 = Imat^(-1)*[0, r4z*upx, -r4y*upx]';
N5 = Imat^(-1)*[-r5y*umz, r5x*umz, 0]';
N6 = Imat^(-1)*[-r6z*upy, 0, r6x*upy]';
f_thrust = N1 + N2 + N3 + N4 + N5 + N6;

Qdot = 1/2*Bq*wbob;
Wdot_Body = f_inert_body + f_tau_gg_body + f_add + f_tau_input_body + ...
    f_thrust;
dX = Fvec + 1/mass*Bmat*force;

DX = [Qdot; Wdot_Body; dX];
end


function S = skewMat(x)

S = [0 -x(3) x(2);
    x(3) 0 -x(1);
    -x(2) x(1) 0];
end

function qout = quatRenorm(qin,tol)

if nargin < 2 || isempty(tol)
    tol = 1e-6;
end

q0 = qin(1); q1 = qin(2); q2 = qin(3); q3 = qin(4);

qn = norm(qin);

if norm(qin) > 1+tol || norm(qin) < 1-tol
    q0 = q0/qn;
    q1 = q1/qn;
    q2 = q2/qn;
    q3 = q3/qn;
    qout = [q0 q1 q2 q3]';
else
    qout = qin;
    return
end
end

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
format long
tol = 1e-10;
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
format short
end