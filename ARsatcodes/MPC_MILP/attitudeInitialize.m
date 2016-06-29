function [Q0,W0] = attitudeInitialize(yaw,pitch,roll,yawRate,pitchRate,rollRate,randflag)

if randflag == 1
    
    yaw = (yaw + 1e0.*randn(1,1))*pi/180;
    pitch = (pitch + 1e0.*randn(1,1))*pi/180;
    roll = (roll + 1e0.*randn(1,1))*pi/180;
    
    yawRate = (yawRate + 1e-1.*randn(1,1))*pi/180;
    pitchRate = (pitchRate + 1e-1.*randn(1,1))*pi/180;
    rollRate = (rollRate + 1e-1.*randn(1,1))*pi/180;
    
elseif randflag == 0
    yaw = (yaw)*pi/180;
    pitch = (pitch)*pi/180;
    roll = (roll)*pi/180;
    
    yawRate = (yawRate)*pi/180;
    pitchRate = (pitchRate)*pi/180;
    rollRate = (rollRate)*pi/180;
else
    
end

EulerAnglesInit = [yaw pitch roll];
EulerAngleRates = [yawRate pitchRate rollRate];
BI_YPR = BinvEuler321(EulerAnglesInit);
Q0 = Euler3212EP(EulerAnglesInit);
Q0 = quatRenorm(Q0,1e-12);
W0 = BI_YPR*EulerAngleRates';
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
