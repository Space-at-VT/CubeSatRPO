function p = plotObstacle(lbnd,ubnd,style)
%% Plot obstacle
xbnd = [lbnd(1) ubnd(1) ubnd(1) lbnd(1) lbnd(1)];
ybnd = [lbnd(2) lbnd(2) ubnd(2) ubnd(2) lbnd(2)];
zlbnd = [lbnd(3) lbnd(3) lbnd(3) lbnd(3) lbnd(3)];
zubnd = [ubnd(3) ubnd(3) ubnd(3) ubnd(3) ubnd(3)];

hold on
p = plot3(ybnd,zlbnd,xbnd,style,'linewidth',1);
plot3(ybnd,zubnd,xbnd,style,'linewidth',1)
for i = 1:4
    plot3([ybnd(i) ybnd(i)],[lbnd(3),ubnd(3)],[xbnd(i) xbnd(i)],style,'linewidth',1)
end
hold off
% grid on
% xlabel('Radial, x [m]')
% ylabel('In-track, y [m]')
% zlabel('Cross-track, z [m]')
% title('Relative Trajectory')
% axis('equal')
% camva(9)
% view(-45,15)
end