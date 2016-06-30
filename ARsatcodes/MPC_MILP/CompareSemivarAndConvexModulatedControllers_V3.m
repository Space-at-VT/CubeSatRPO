yalmip('clear');
clear;
close all;
clc;

FTS = load('MILP_FullyActTrajGen_TrajectoryPlanner_tf_333.mat');
Shuttle = load('shuttleStructV2.mat');
[Xk,Uk,Time,Ts,Nsim,Xt0,Xf,~,~,Umax,Ub,m0,mass,Xqo,Uqo,Isp] = ...
    unpackManeuver(FTS);
Req = 6378.1363e3;
mu = 3.986e14;
J2 = 1082.629e-6;
a = 6678e3;
ecc = 0.01;
inc = 28.5*pi/180;
raan = pi/4;
argper = pi/4;
f0 = 0;
ChiefElems = [a ecc inc raan argper f0];

n = sqrt(mu/a^3);
period = 2*pi/n;
Nt = 25;
tLoiter = 10;
0;
timeHold = Time(end)+1:Ts:Time(end)+tLoiter;

Time = [Time, timeHold, timeHold(end)+1];
Ref = [Xk(1,:); Xk(2,:); Xk(3,:); Xk(4,:); Xk(5,:); Xk(6,:)];
Ref = [Ref, repmat(Ref(:,end),1,Nt+length(timeHold))];
Uk2 = [Uk, zeros(3,Nt+length(timeHold+1)); zeros(3,length(Uk)), zeros(3,Nt+length(timeHold+1))];
Uk = [Uk, zeros(3,length(timeHold))];


g0 = 9.806;
[fuelCostOptimal,deltaVOptimal,mOpt] = TotalFuel(Time(1:end-1),Uk,m0,...
    Isp,g0,Ts);
mOpt = [mOpt, repmat(mOpt(end),1,length(tLoiter))];

minImpBit = 0.99*Umax;
Umin = minImpBit;
Imat = diag([0.140998 0.10559 0.05417]);
Nr = 25;

maxTorque = 0.635e-3;
minBit = 0.0*maxTorque;

r1x = 0.01; r1z = 0.01;
r2x = -0.01; r2y = 0.02;
r3y = -0.01; r3z = -0.01;
r4y = 0.02; r4z = 0.02;
r5x = -0.01; r5y = -0.002;
r6x = 0.009; r6z = 0.002;
rho = [r1x r1z r2x r2y r3y r3z r4y r4z r5x r5y r6x r6z];

[Ac,Bc] = getCoupledSystemJacobians(Imat,m0,n,rho);
Cc = eye(13);
sysc = ss(Ac,Bc,[],[]);
sysd = c2d(sysc,Ts,'zoh');

Ac = sysd.a;
Bc = sysd.b;

At = Ac(8:end,8:end);
Bt = Bc(8:end,4:end);
Arot = Ac(2:7,2:7);
Brot = Bc(2:7,1:3);
Ct = eye(6);

nx = 6;
nr = 6;
nu = 6;
ntau = 3;
ny = 6;

Qtmi = eye(nx);
Rtmi = 1e3.*eye(nu);
Qrmi = eye(nr);
Rrmi = 1e4.*eye(ntau);

Qtmod = eye(nx);
Rtmod = 5*1e0.*eye(nu);
% Rtmod = 1e3.*eye(nu);
Qrmod = eye(nr);
Rrmod = 1e4.*eye(ntau);

% yaw0 = 10;
% pitch0 = -20;
% roll0 = 10;
% yawRate0 = 1;
% pitchRate0 = 0;
% rollRate0 = -3;
yaw0 = 0;
pitch0 = 0;
roll0 = 0;
yawRate0 = 0;
pitchRate0 = 0;
rollRate0 = 0;
options = sdpsettings('solver','gurobi','verbose',0,'gurobi.TimeLimit',Ts);
% Convert initial attitude errors into quaternion/angular velocity errors
[Q0,W0] = attitudeInitialize(yaw0,pitch0,roll0,yawRate0,pitchRate0,...
    rollRate0,0);
% get rid of redundant quaternion value (the scalar value)
Xr0 = [Q0(2:4); W0];
small = 0.0.*Xt0;
xt = Xt0 - small;
d = [Xr0; xt];

% l_p norms
pnorm = [1 2 1 2];

for ii = 1:length(pnorm)
    if ii == 1 || ii == 2
        [TrackingController,AttitudeController] = ...
            buildSatelliteMPC_SemivarU_V2(Nt,At,Bt,Ct,Qtmi,Rtmi,Umax,Umin,...
            Shuttle,maxTorque,Arot,Brot,Nr,Qrmi,Rrmi,timeHold,pnorm(ii),options);
        [X(:,:,ii),err(:,:,ii),yaw(:,ii),pitch(:,ii),roll(:,ii),...
            yawdot(:,ii),pitchdot(:,ii),rolldot(:,ii),Q(:,:,ii),Thruster(:,:,ii),...
            Torques(:,:,ii),deltaVTracking(ii),mTrack(:,ii),Xqt(:,:,ii),...
            Uqt(:,:,ii),rotEnergy(ii),tranEnergy(ii),trajDiag(:,ii),attDiag(:,ii),...
            trajObj(:,ii),attObj(:,ii),punt(ii),pctPunt(ii)] = ...
            TrackTraj_MPC_with_Thrust_Torques_V2(Time,d,Nt,Ref,Uk2,...
            TrackingController,AttitudeController,nx,nu,ny,ntau,Nsim,Isp,...
            g0,m0,Ts,Imat,a,ecc,mu,n,rho);
    elseif ii == 3 || ii == 4
        [TrackingController,AttitudeController] = ...
            buildSatelliteMPC_V3(Nt,At,Bt,Ct,Qtmod,Rtmod,Umax,...
            Shuttle,maxTorque,Arot,Brot,Nr,Qrmod,Rrmod,timeHold,pnorm(ii),options);
        [X(:,:,ii),err(:,:,ii),yaw(:,ii),pitch(:,ii),roll(:,ii),...
            yawdot(:,ii),pitchdot(:,ii),rolldot(:,ii),Q(:,:,ii),Thruster(:,:,ii),...
            Torques(:,:,ii),deltaVTracking(ii),mTrack(:,ii),Xqt(:,:,ii),...
            Uqt(:,:,ii),rotEnergy(ii),tranEnergy(ii),trajDiag(:,ii),attDiag(:,ii),...
            trajObj(:,ii),attObj(:,ii),punt(ii),pctPunt(ii)] = ...
            TrackTraj_MPC_with_Thrust_Torques_Modulated_V2(Time,d,Nt,Ref,Uk2,...
            TrackingController,AttitudeController,nx,nu,ny,ntau,Nsim,Isp,...
            g0,m0,Ts,Imat,a,ecc,mu,n,rho,Umin);
    else
    end  
end

for ii = 1:length(Uk)
    ukIP(ii) = dot(Uk(:,ii),Uk(:,ii));
    ukn(ii) = norm(Uk(:,ii));
end

optEnergy = trapz(Time,ukIP);
optTI = totalImpulse(Time,ukn);

for jj = 1:length(pnorm)
    for ii = 1:length(Uk)
        uk(ii,jj) = norm(Thruster(:,ii,jj));
        rk(ii,jj) = norm(Torques(:,ii,jj));
    end
    ukti(jj) = totalImpulse(Time,uk(:,jj));
    rkti(jj) = totalImpulse(Time,rk(:,jj));
end

% struct.Norm_Type = {'Opt';'1-MI';'2-MI';'1-Mod';'2-Mod'};
% struct.Delta_V_Total = [deltaVOptimal; deltaVTracking(1); ...
%     deltaVTracking(2); deltaVTracking(3); deltaVTracking(4)];
% struct.Thrust_Energy = [optEnergy; tranEnergy(1); tranEnergy(2);...
%     tranEnergy(3); tranEnergy(4)];
% struct.Total_Impulse = [optTI; ukti(1); ukti(2); ukti(3); ukti(4)];
% struct.Rotation_Energy = [0 ; rotEnergy(1); rotEnergy(2); rotEnergy(3); ...
%     rotEnergy(4)];
% struct.Rotation_Impulse = [0; rkti(1); rkti(2); rkti(3); rkti(4)];
% struct.Number_of_Punts = [0; punt(1); punt(2); punt(3); punt(4)];
% struct.Percent_Punts = [0; pctPunt(1); pctPunt(2); pctPunt(3); pctPunt(4)];
% T = struct2table(struct);
% save(['mimpcstruct_tf_' num2str(Time(end)) '_noICerr.mat'],'struct');
% disp(T)


plotSimulationsV2(Time,X,Ref,Xt0,Xf,Uk,err,Thruster,Umax,yaw,pitch,roll,...
    Torques,maxTorque,mOpt,mTrack,deltaVOptimal,deltaVTracking)

save compareworkspace_18_dec_tf_1232_noICerr.mat
