function [dv,dx] = rk4(sat)
x = sat.p;
v = sat.v;
dt = sat.scenario.dt;

x1 = x;
v1 = v;
a1 = a(sat,x1,v1,0);

x2 = x1+v1*dt/2;
v2 = v1+a1*dt/2;
a2 = a(sat,x2,v2,dt/2);

x3 = x2+v2*dt/2;
v3 = v2+a2*dt/2;
a3 = a(sat,x3,v3,dt/2);

x4 = x3+v3*dt;
v4 = v3+a3*dt;
a4 = a(sat,x4,v4,dt);

dv = 1/6*(a1+2*a2+2*a3+a4)*dt;
dx = 1/6*(v1+2*v2+2*v3+v4)*dt;
end

function acc = a(sat,x,v,dt)
switch sat.EOM
    case 'HCW'
        A = HCW(sat.scenario);
    case 'LERM'
        A = LERM(sat.scenario,sat.t(end)+dt);
end
X = [x,v]';
DX = A*X;
acc = DX(4:6)'+[sat.ux(end),sat.uy(end),sat.uz(end)]/sat.m;
end