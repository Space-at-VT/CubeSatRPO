function plotTrajectory(sat,lbnd,ubnd,unit)
if nargin < 2 || isempty(unit),unit = 5;end
fig = figure(1);

% Plot obstacle bounds
for ii = 1:size(lbnd,1)
    plotObstacle(lbnd(ii,:),ubnd(ii,:),'-k');
end

% Plot satellite trajectory
hold on
for jj = 1:length(sat)
    p1style = strcat('-',sat.color);
    p4style = 'ks';
    plot3(sat.y,sat.z,sat.x,p1style,'linewidth',1.5);
    quiver3(sat.y,sat.z,sat.x,-sat.uy,-sat.uz,-sat.ux,1,'r','linewidth',1.5);
    plot3(sat.p(2),sat.p(3),sat.p(1),p4style,'linewidth',2,'markersize',10);
    
    % Plot satellite body axes
    R = unit*sat.Rib;
    plot3([sat.p(2),sat.p(2)+R(2,1)'],[sat.p(3),sat.p(3)+R(3,1)'],...
        [sat.p(1),sat.p(1)+R(1,1)'],'b','linewidth',1.5);
    plot3([sat.p(2),sat.p(2)+R(2,2)'],[sat.p(3),sat.p(3)+R(3,2)'],...
        [sat.p(1),sat.p(1)+R(1,2)'],'r','linewidth',1.5);
    plot3([sat.p(2),sat.p(2)+R(2,3)'],[sat.p(3),sat.p(3)+R(3,3)'],...
        [sat.p(1),sat.p(1)+R(1,3)'],'g','linewidth',1.5);
end
hold off

% Axis labels
grid on
zlabel('Radial, x [m]')
xlabel('In-track, y [m]')
ylabel('Cross-track, z [m]')
title('Relative Trajectory')
axis('tight','equal','vis3d')
camva(10)
% axis('equal','vis3d')
view(145,15)

% Save movie
if sat.makeMovie
    sat = sat.addFrame(fig);
end

drawnow
end
