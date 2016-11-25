function [ dcm ] = q2DCM( q )
%q2DCM Given the quaternion, determines the direction cosine matrix
qbar = q(1:3);

dcm = (q(4)^2-(qbar'*qbar))*eye(3)+2*(qbar*qbar')-2*q(4)*skew(qbar);


end

