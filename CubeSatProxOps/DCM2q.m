function [ q ] = DCM2q( dcm )
%DCM2q Given Direction Cosine Matrix, provides quaternions
q = zeros(4,1);
q(4) = 0.5*sqrt(1+dcm(1,1)+dcm(2,2)+dcm(3,3));
q(1) = (dcm(2,3)-dcm(3,2))/(4*q(4));
q(2) = (dcm(3,1)-dcm(1,3))/(4*q(4));
q(3) = (dcm(1,2)-dcm(2,1))/(4*q(4));
q = q/norm(q); %normalize b/c of numerical error
end

