function [ output_args ] = gibbsOrbitDeterm( R, T )
%gibbsOrbitDeterm Finds the orbital parameters given three position vectors
%and the time at which they were found
%   INPUTS:
%       R - Three position vectors -> [R1, R2, R3], in ECI frame
%       T - Three times -> [t1, t2, t3]

%Find time differentials:
tau1 = T(1)-T(2);
tau3 = T(3)-T(2);
tau = T(3)-T(1);

%Find cross products of position unit vectors;
R1 = R(:,1);
R2 = R(:,2);
R3 = R(:,3);

rho1 = R1/norm(R1);
rho2 = R2/norm(R2);
rho3 = R3/norm(R3);

p1 = cross(rho2, rho3);
p2 = cross(rho1, rho3);
p3 = cross(rho1, rho2);

%Find common scalar quantity
D0 = dot(rho1,p1);

D = [

end

