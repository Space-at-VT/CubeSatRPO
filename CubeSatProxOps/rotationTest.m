clear,clc
close all

up = [1,0,0]';

th1 = 90*pi/180;
th2 = 0*pi/180;
th3 = 0*pi/180;

R = rot(th3,3)*rot(th2,1)*rot(th1,3);
u = R\up;

%%
clear

p0 = [0,0,0];
pt = [1,0,1];
vt = (pt-p0)/norm(pt-p0)

x = pt-p0;

th3 = atan2d(vt(2),vt(1))
th2 = atan2d(vt(3),norm([vt(1),vt(2)]))

R = rot(th2*pi/180,2)*rot(th3*pi/180,3)

q4 = 1/2*sqrt(1+trace(R));
q = 1/4/q4*[R(2,3)-R(3,2);R(3,1)-R(1,3);R(1,2)-R(2,1)];

qb = [q;q4]
% 
% q = qb(1:3);
% q4 = qb(4);
% qx = [0 -q(3) q(2)
%     q(3) 0 -q(1)
%     -q(2) q(1) 0];
% Rbi = (q4^2-q'*q)*eye(3)+2*(q*q')-2*q4*qx
% R = Rbi'