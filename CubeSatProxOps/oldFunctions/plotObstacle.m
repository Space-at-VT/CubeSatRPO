function p = plotObstacle(lbnd,ubnd,style)
%% Plot obstacle
xbnd = [lbnd(1) ubnd(1) ubnd(1) lbnd(1) lbnd(1)];
ybnd = [lbnd(2) lbnd(2) ubnd(2) ubnd(2) lbnd(2)];
zlbnd = [lbnd(3) lbnd(3) lbnd(3) lbnd(3) lbnd(3)];
zubnd = [ubnd(3) ubnd(3) ubnd(3) ubnd(3) ubnd(3)];

hold on
p = plot3(xbnd,ybnd,zlbnd,style,'linewidth',1);
plot3(xbnd,ybnd,zubnd,style,'linewidth',1)
for ii = 1:4
    plot3([xbnd(ii) xbnd(ii)],[ybnd(ii),ybnd(ii)],[lbnd(3) ubnd(3)],style,'linewidth',1)
end
hold off
end