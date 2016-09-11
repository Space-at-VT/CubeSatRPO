function p1 = plotTrajectory(sat)
p1style = strcat(':',sat.color);
p2style = strcat(sat.color,'^');
p4style = 'ks';

figure(1)
hold on
p1 = plot3(sat.y,sat.z,sat.x,p1style,'linewidth',2);
% p2 = plot3(sat.y(1),sat.z(1),sat.x(1),p2style,'linewidth',2,'markersize',10);
% p3 = quiver3(sat.y,sat.z,sat.x,-sat.uy,-sat.uz,-sat.ux,1,'r','linewidth',2);
p4 = plot3(sat.p(2),sat.p(3),sat.p(1),p4style,'linewidth',2,'markersize',10);

R = 5*rot(sat.th3(end),3)*rot(sat.th2(end),1)*rot(sat.th1(end),3);

r1 = plot3([sat.p(2),sat.p(2)+R(2,1)'],[sat.p(3),sat.p(3)+R(3,1)'],...
    [sat.p(1),sat.p(1)+R(1,1)'],'b','linewidth',2);
r2 = plot3([sat.p(2),sat.p(2)+R(2,2)'],[sat.p(3),sat.p(3)+R(3,2)'],...
    [sat.p(1),sat.p(1)+R(1,2)'],'r','linewidth',2);
r3 = plot3([sat.p(2),sat.p(2)+R(2,3)'],[sat.p(3),sat.p(3)+R(3,3)'],...
    [sat.p(1),sat.p(1)+R(1,3)'],'g','linewidth',2);

hold off
grid on
zlabel('Radial, x [m]')
xlabel('In-track, y [m]')
ylabel('Cross-track, z [m]')
title('Relative Trajectory')
axis('tight','equal')
camva(9)
view(145,15)
end
