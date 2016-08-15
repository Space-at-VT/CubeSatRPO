function [ p, e, a ] = gibbsOrbitDeterm( R, T )
%gibbsOrbitDeterm Finds the orbital parameters given three position vectors
%and the time at which they were found
% ERROR OF 0.6111 km/s WITH 1 SECOND SPACING
%   INPUTS:
%       R - Three position vectors -> [R1, R2, R3], in ECI frame
%       T - Three times -> [t1, t2, t3]

mu = 3.9860004418*10^14;

%Break up R into position vectors;
R1 = R(:,1);
R2 = R(:,2);
R3 = R(:,3);

%Normalize position vectors
r1=norm(R1);
r2=norm(R2);
r3=norm(R3);

%Find D, N, and S
D = cross(R1,R2)+cross(R2,R3)+cross(R3,R1);
N = r3*cross(R1,R2)+r1*cross(R2,R3)+r2*cross(R3,R1);
S = (r2-r3)*R1+(r3-r1)*R2+(r1-r2)*R3;

d = norm(D);
n = norm(N);
s = norm(S);

%Find ellipse parameters
p=n/d;
e=s/d;
a=p/(1-e^2);

%Find PQW frame
Q=S/s;
W=N/n;
P=cross(Q,W);

%Find transformation from ECI to PQW
T = [P Q W];

l=sqrt(mu/(d*n));
B1=cross(D,R1);
B2=cross(D,R2);
B3=cross(D,R3);

V1=(l/r1)*B1+l*S
V2=(l/r2)*B2+l*S
V3=(l/r3)*B3+l*S


end

