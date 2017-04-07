clear,clc
close all

c1 = [0,0.4470,0.7410];
c2 = [0.9290,0.6940,0.1250];

plume = 1;

figure('Position',[100 100 1280 720]);
hold on

rso = plot3(0,0,0,'ko','markerfacecolor','b','markersize',6);

load('throttle')
p0 = plot3(sat.x(1),sat.y(1),sat.z(1),'ko','markerfacecolor','g','markersize',6);
pf = plot3(sat.x(end),sat.y(end),sat.z(end),'ko','markerfacecolor','r','markersize',6);
trj1 = plot3(sat.x,sat.y,sat.z,'-','linewidth',1,'color',c1);
pl = quiver3(sat.x,sat.y,sat.z,-sat.ux,-sat.uy,-sat.uz,plume,'r','linewidth',1.5);
sat.fuelUsed

load('nothrottle')
p0 = plot3(sat.x(1),sat.y(1),sat.z(1),'ko','markerfacecolor','g','markersize',6);
pf = plot3(sat.x(end),sat.y(end),sat.z(end),'ko','markerfacecolor','r','markersize',6);
trj2 = plot3(sat.x,sat.y,sat.z,'-','linewidth',1,'color',c2);
pl = quiver3(sat.x,sat.y,sat.z,-sat.ux,-sat.uy,-sat.uz,plume,'r','linewidth',1.5);
sat.fuelUsed
hold off

grid on
xl = xlabel('Radial, m');
yl = ylabel('In-track, m');
zl = zlabel('Cross-track, m');
lg = legend([rso,p0,pf,trj1,trj2,pl],{'RSO','Initial Position','Final Position',...
    'Relative Trajectory','Relative Trajectory, Binary Throttle','Thrust Plume'},'location','northoutside','orientation','horizontal');

set([xl,yl,zl,lg],'FontSize',12)
axis('equal','tight')
camva(6)
drawnow