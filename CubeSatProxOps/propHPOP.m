clear,clc
close all

app = actxserver('STK11.application');
root = app.Personality2;
app.visible = 0;

% Initial state
umax = 10;
u = [1,0,0];
x = 10;
y = 0;
z = 0;
vx = 0;
vy = 0;
vz = 0;
dt = 1;

X = [x,y,z,vx,vy,vz]';

for ii = 1:100
    X = HPOP(root,X,u,umax,dt)
    x(ii) = X(1);
    y(ii) = X(2);
    z(ii) = X(3);
end

plot3(x,y,z,'linewidth',2)
grid on
axis('equal','vis3d')
