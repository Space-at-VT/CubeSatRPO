clear,clc
close all

app = actxserver('STK11.application');
root = app.Personality2;
app.visible = 0;

% Initial state
u = [-1,0,0];
x = 50;
y = 1;
z = 1;
vx = 0;
vy = 0;
vz = 0;
dt = 1;

X = [x,y,z,vx,vy,vz]';

for ii = 1:1000;
    X = HPOP(root,X,u,dt)
end
