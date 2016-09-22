function [dxdt,dvdt] = rk(x,v,a,dt)
if isempty(a),a = @(x,v)0;end
x1 = x;
v1 = v;
a1 = a(x1,v1);

x2 = x1+v1*dt/2;
v2 = v1+a1*dt/2;
a2 = a(x2,v2);

x3 = x2+v2*dt/2;
v3 = v2+a2*dt/2;
a3 = a(x3,v3);

x4 = x3+v3*dt;
v4 = v3+a3*dt;
a4 = a(x4,v4);

dvdt = 1/6*(a1+2*a2+2*a3+a4);
dxdt = 1/6*(v1+2*v2+2*v3+v4);
end

