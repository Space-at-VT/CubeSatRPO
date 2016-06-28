function plotSimulations(Time,X,Ref,Xt0,Xf,Uk,err,Thruster,Umax,yaw,pitch,...
    roll,Torques,maxTorque,mOpt,mTrack,deltaVOptimal,deltaVTracking)

fig1 = figure(1);
hold on
h1 = plot3(X(1,:,1),X(2,:,1),X(3,:,1),'r','LineWidth',1.5);
h2 = plot3(X(1,:,3),X(2,:,3),X(3,:,3),'b','LineWidth',1.5);
h4 = plot3(Ref(1,:),Ref(2,:),Ref(3,:),'k-','LineWidth',2);
h5 = plot3(Xt0(1),Xt0(2),Xt0(3),'b.','MarkerSize',20);
h6 = plot3(Xf(1),Xf(2),Xf(3),'k.','MarkerSize',20);
h7 = plot3(X(1,1,1),X(2,1,1),X(3,1,1),'kd','MarkerFaceColor','k',...
    'MarkerSize',6);
PlotShuttleApproximationsV2(0,0,1)
title1 = title('Motion in 3-Dimensional Space- 1-Norm');
leg1 = legend([h1 h2 h4 h5 h6 h7],'$\textbf{x}_{MI}(t)$','$\textbf{x}_{Mod}(t)$',...
    '$\textbf{r}(t)$',...
    '$\textbf{x}_0$','$\textbf{x}_f$','$\textbf{x}_0$ Actual',...
    'Location','Best');
set(leg1,'interpreter','latex','fontsize',10);
xl = xlabel('Radial, $x$, m');
yl = ylabel('In-track, $y$, m');
zl = zlabel('Cross-track, $z$, m');
set([title1 xl yl zl],'interpreter','latex','fontsize',12)
set(leg1,'interpreter','latex','fontsize',8);
axis tight
saveas(fig1,['mimpcfigures/1norm3d_tf_' num2str(Time(end)) '_noICerr.fig'])

fig2 = figure(2);
hold on
h1 = plot3(X(1,:,2),X(2,:,2),X(3,:,2),'r','LineWidth',1.5);
h2 = plot3(X(1,:,4),X(2,:,4),X(3,:,4),'b','LineWidth',1.5);
h4 = plot3(Ref(1,:),Ref(2,:),Ref(3,:),'k-','LineWidth',2);
h5 = plot3(Xt0(1),Xt0(2),Xt0(3),'b.','MarkerSize',20);
h6 = plot3(Xf(1),Xf(2),Xf(3),'k.','MarkerSize',20);
h7 = plot3(X(1,1,1),X(2,1,1),X(3,1,1),'kd','MarkerFaceColor','k',...
    'MarkerSize',6);
PlotShuttleApproximationsV2(0,0,1)
title1 = title('Motion in 3-Dimensional Space- 2-Norm');
leg1 = legend([h1 h2 h4 h5 h6 h7],'$\textbf{x}_{MI}(t)$','$\textbf{x}_{Mod}(t)$',...
    '$\textbf{r}(t)$',...
    '$\textbf{x}_0$','$\textbf{x}_f$','$\textbf{x}_0$ Actual',...
    'Location','Best');
set(leg1,'interpreter','latex','fontsize',10);
xl = xlabel('Radial, $x$, m');
yl = ylabel('In-track, $y$, m');
zl = zlabel('Cross-track, $z$, m');
set([title1 xl yl zl],'interpreter','latex','fontsize',12)
set(leg1,'interpreter','latex','fontsize',8);
axis tight
saveas(fig2,['mimpcfigures/2norm3d_tf_' num2str(Time(end)) '_noICerr.fig'])

fig3 = figure(3);
subplot(311)
hold on
h1 = plot(Time,err(1,:,1),'r','LineWidth',1);
h2 = plot(Time,err(1,:,3),'b','LineWidth',1);
h4 = plot([1 Time(end)],[0 0],'k--','LineWidth',1);
leg1 = legend([h1 h2],'$\textbf{x}_{MI}(t)$','$\textbf{x}_{Mod}(t)$',...
    'Location','Best');
title1 = title('Tracking Error, $\textbf{x}_k - \textbf{r}_k$, 1-Norm');
yl1 = ylabel('Radial');
axis tight
subplot(312)
hold on
plot(Time,err(2,:,1),'r','LineWidth',1);
plot(Time,err(2,:,3),'b','LineWidth',1);
plot([1 Time(end)],[0 0],'k--','LineWidth',1)
yl2 = ylabel('In-track');
axis tight
subplot(313)
hold on
plot(Time,err(3,:,1),'r','LineWidth',1);
plot(Time,err(3,:,3),'b','LineWidth',1);
plot([1 Time(end)],[0 0],'k--','LineWidth',1)
yl3 = ylabel('Cross-track');
xl = xlabel('Time, $t$, s');
set([title1 yl1 yl2 yl3 xl],'interpreter','latex','fontsize',12)
set(leg1,'interpreter','latex','fontsize',12);
axis tight
saveas(fig3,['mimpcfigures/1normerr_tf_' num2str(Time(end)) '_noICerr.fig'])

fig4 = figure(4);
subplot(311)
hold on
h1 = plot(Time,err(1,:,2),'r','LineWidth',1);
h2 = plot(Time,err(1,:,4),'b','LineWidth',1);
h4 = plot([1 Time(end)],[0 0],'k--','LineWidth',1);
leg1 = legend([h1 h2],'$\textbf{x}_{MI}(t)$','$\textbf{x}_{Mod}(t)$',...
    'Location','Best');
title1 = title('Tracking Error, $\textbf{x}_k - \textbf{r}_k$, 2-Norm');
yl1 = ylabel('Radial');
axis tight
subplot(312)
hold on
plot(Time,err(2,:,2),'r','LineWidth',1);
plot(Time,err(2,:,4),'b','LineWidth',1);
plot([1 Time(end)],[0 0],'k--','LineWidth',1)
yl2 = ylabel('In-track');
axis tight
subplot(313)
hold on
plot(Time,err(3,:,2),'r','LineWidth',1);
plot(Time,err(3,:,4),'b','LineWidth',1);
plot([1 Time(end)],[0 0],'k--','LineWidth',1)
yl3 = ylabel('Cross-track');
xl = xlabel('Time, $t$, s');
set([title1 yl1 yl2 yl3 xl],'interpreter','latex','fontsize',12)
set(leg1,'interpreter','latex','fontsize',12);
axis tight
saveas(fig4,['mimpcfigures/2normerr_tf_' num2str(Time(end)) '_noICerr.fig'])

fig5 = figure(5);
subplot(311)
hold on
h1 = plot(Time,err(4,:,1),'r','LineWidth',1);
h2 = plot(Time,err(4,:,3),'b','LineWidth',1);
h4 = plot([1 Time(end)],[0 0],'k--','LineWidth',1);
leg1 = legend([h1 h2],'$\textbf{x}_{MI}(t)$','$\textbf{x}_{Mod}(t)$',...
    'Location','Best');
title1 = title('Tracking Error Rates, 1-Norm');
yl1 = ylabel('Radial');
axis tight
subplot(312)
hold on
plot(Time,err(5,:,1),'r','LineWidth',1);
plot(Time,err(5,:,3),'b','LineWidth',1);
plot([1 Time(end)],[0 0],'k--','LineWidth',1)
yl2 = ylabel('In-track');
axis tight
subplot(313)
hold on
plot(Time,err(6,:,1),'r','LineWidth',1);
plot(Time,err(6,:,3),'b','LineWidth',1);
plot([1 Time(end)],[0 0],'k--','LineWidth',1)
yl3 = ylabel('Cross-track');
xl = xlabel('Time, $t$, s');
set([title1 yl1 yl2 yl3 xl],'interpreter','latex','fontsize',12)
set(leg1,'interpreter','latex','fontsize',12);
axis tight
saveas(fig5,['mimpcfigures/1normerrv_tf_' num2str(Time(end)) '_noICerr.fig'])

fig6 = figure(6);
subplot(311)
hold on
h1 = plot(Time,err(4,:,2),'r','LineWidth',1);
h2 = plot(Time,err(4,:,4),'b','LineWidth',1);
h4 = plot([1 Time(end)],[0 0],'k--','LineWidth',1);
leg1 = legend([h1 h2],'$\textbf{x}_{MI}(t)$','$\textbf{x}_{Mod}(t)$',...
    'Location','Best');
title1 = title('Tracking Error Rates, 2-Norm');
yl1 = ylabel('Radial');
axis tight
subplot(312)
hold on
plot(Time,err(5,:,2),'r','LineWidth',1);
plot(Time,err(5,:,4),'b','LineWidth',1);
plot([1 Time(end)],[0 0],'k--','LineWidth',1)
yl2 = ylabel('In-track');
axis tight
subplot(313)
hold on
plot(Time,err(6,:,2),'r','LineWidth',1);
plot(Time,err(6,:,4),'b','LineWidth',1);
plot([1 Time(end)],[0 0],'k--','LineWidth',1)
yl3 = ylabel('Cross-track');
xl = xlabel('Time, $t$, s');
set([title1 yl1 yl2 yl3 xl],'interpreter','latex','fontsize',12)
set(leg1,'interpreter','latex','fontsize',12);
axis tight
saveas(fig6,['mimpcfigures/2normerrv_tf_' num2str(Time(end)) '_noICerr.fig'])

fig7 = figure(7);
subplot(311)
hold on
h1 = stairs(Time,Thruster(1,:,1),'r','LineWidth',1);
h2 = stairs(Time,Thruster(1,:,3),'b','LineWidth',1);
h4 = stairs(Time(1:end-1),Uk(1,:),'k-','LineWidth',2);
h5 = plot([1 Time(end)],[Umax Umax],'k--');
title1 = title('Thruster History, 1-Norm');
plot([1 Time(end)],[-Umax -Umax],'k--')
axis([1,Time(end),-2*Umax,2*Umax])
leg1 = legend([h1 h2 h4 h5],'$u_{MI}(t)$','$u_{Mod}(t)$','Optimal $u(t)$',...
    '$u_{max}$','Location','Best');
yl1 = ylabel('Radial');
subplot(312)
hold on
stairs(Time,Thruster(2,:,1),'r','LineWidth',1);
stairs(Time,Thruster(2,:,3),'b','LineWidth',1);
stairs(Time(1:end-1),Uk(2,:),'k-','LineWidth',2)
plot([1 Time(end)],[Umax Umax],'k--')
plot([1 Time(end)],[-Umax -Umax],'k--')
axis([1,Time(end),-2*Umax,2*Umax])
yl2 = ylabel('In-track');
subplot(313)
hold on
stairs(Time,Thruster(3,:,1),'r','LineWidth',1);
stairs(Time,Thruster(3,:,3),'b','LineWidth',1);
stairs(Time(1:end-1),Uk(3,:),'k-','LineWidth',2)
plot([1 Time(end)],[Umax Umax],'k--')
plot([1 Time(end)],[-Umax -Umax],'k--')
axis([1,Time(end),-2*Umax,2*Umax])
yl3 = ylabel('Cross-track');
xl = xlabel('Time, $t$, s');
set([title1 yl1 yl2 yl3 xl],'interpreter','latex','fontsize',12)
set(leg1,'interpreter','latex','fontsize',8);
saveas(fig7,['mimpcfigures/1normthruster_tf_' num2str(Time(end)) '_noICerr.fig'])

fig8 = figure(8);
subplot(311)
hold on
h1 = stairs(Time,Thruster(1,:,2),'r','LineWidth',1);
h2 = stairs(Time,Thruster(1,:,4),'b','LineWidth',1);
h4 = stairs(Time(1:end-1),Uk(1,:),'k-','LineWidth',2);
h5 = plot([1 Time(end)],[Umax Umax],'k--');
title1 = title('Thruster History, 2-Norm');
plot([1 Time(end)],[-Umax -Umax],'k--')
axis([1,Time(end),-2*Umax,2*Umax])
leg1 = legend([h1 h2 h4 h5],'$u_{MI}(t)$','$u_{Mod}(t)$','Optimal $u(t)$',...
    '$u_{max}$','Location','Best');
yl1 = ylabel('Radial');
subplot(312)
hold on
stairs(Time,Thruster(2,:,2),'r','LineWidth',1);
stairs(Time,Thruster(2,:,4),'b','LineWidth',1);
stairs(Time(1:end-1),Uk(2,:),'k-','LineWidth',2)
plot([1 Time(end)],[Umax Umax],'k--')
plot([1 Time(end)],[-Umax -Umax],'k--')
axis([1,Time(end),-2*Umax,2*Umax])
yl2 = ylabel('In-track');
subplot(313)
hold on
stairs(Time,Thruster(3,:,2),'r','LineWidth',1);
stairs(Time,Thruster(3,:,4),'b','LineWidth',1);
stairs(Time(1:end-1),Uk(3,:),'k-','LineWidth',2)
plot([1 Time(end)],[Umax Umax],'k--')
plot([1 Time(end)],[-Umax -Umax],'k--')
axis([1,Time(end),-2*Umax,2*Umax])
yl3 = ylabel('Cross-track');
xl = xlabel('Time, $t$, s');
set([title1 yl1 yl2 yl3 xl],'interpreter','latex','fontsize',12)
set(leg1,'interpreter','latex','fontsize',8);
saveas(fig8,['mimpcfigures/2normthruster_tf_' num2str(Time(end)) '_noICerr.fig'])

fig9 = figure(9);
subplot(311)
hold on
h1 = plot(Time,yaw(:,1),'r','LineWidth',1);
h2 = plot(Time,yaw(:,3),'b','LineWidth',1);
title1 = title('3-2-1 Euler Angle History, 1-Norm');
leg1 = legend([h1 h2],'MI','Mod.','Location','Best');
yl1 = ylabel('Yaw, $\psi(t)$, deg');
axis tight
subplot(312)
hold on
plot(Time,pitch(:,1),'r','LineWidth',1)
plot(Time,pitch(:,3),'b','LineWidth',1)
yl2 = ylabel('Pitch, $\theta(t)$, deg');
axis tight
subplot(313)
hold on
plot(Time,roll(:,1),'r','LineWidth',1)
plot(Time,roll(:,3),'b','LineWidth',1)
yl3 = ylabel('Roll, $\phi(t)$, deg');
xl = xlabel('Time, $t$, s');
set([title1 yl1 yl2 yl3 xl],'interpreter','latex','fontsize',12)
set(leg1,'interpreter','latex','fontsize',8);
axis tight
saveas(fig9,['mimpcfigures/1normeuler_tf_' num2str(Time(end)) '_noICerr.fig'])

fig10 = figure(10);
subplot(311)
hold on
h1 = plot(Time,yaw(:,2),'r','LineWidth',1);
h2 = plot(Time,yaw(:,3),'b','LineWidth',1);
title1 = title('3-2-1 Euler Angle History, 2-Norm');
leg1 = legend([h1 h2],'MI','Mod.','Location','Best');
yl1 = ylabel('Yaw, $\psi(t)$, deg');
axis tight
subplot(312)
hold on
plot(Time,pitch(:,2),'r','LineWidth',1)
plot(Time,pitch(:,4),'b','LineWidth',1)
yl2 = ylabel('Pitch, $\theta(t)$, deg');
axis tight
subplot(313)
hold on
plot(Time,roll(:,2),'r','LineWidth',1)
plot(Time,roll(:,4),'b','LineWidth',1)
yl3 = ylabel('Roll, $\phi(t)$, deg');
xl = xlabel('Time, $t$, s');
set([title1 yl1 yl2 yl3 xl],'interpreter','latex','fontsize',12)
set(leg1,'interpreter','latex','fontsize',8);
axis tight
saveas(fig10,['mimpcfigures/2normeuler_tf_' num2str(Time(end)) '_noICerr.fig'])

fig11 = figure(11);
subplot(311)
hold on
h1 = plot(Time,Torques(1,:,1),'r','LineWidth',1);
h2 = plot(Time,Torques(1,:,3),'b','LineWidth',1);
h3 = plot([Time(1) Time(end)],[maxTorque maxTorque],'k--','LineWidth',2);
plot([Time(1) Time(end)],[-maxTorque -maxTorque],'k--','LineWidth',2)
axis([Time(1) Time(end) -2*maxTorque 2*maxTorque])
title1 = title('Torque History, 1-Norm');
leg1 = legend([h1 h2 h3],'MI','Mod.','Max. Torque',...
    'Location','Best');
yl1 = ylabel('$\tau_1$, mNm');
subplot(312)
hold on
plot(Time,Torques(2,:,1),'r','LineWidth',1);
plot(Time,Torques(2,:,3),'b','LineWidth',1);
plot([Time(1) Time(end)],[maxTorque maxTorque],'k--','LineWidth',2)
plot([Time(1) Time(end)],[-maxTorque -maxTorque],'k--','LineWidth',2)
axis([Time(1) Time(end) -2*maxTorque 2*maxTorque])
yl2 = ylabel('$\tau_2$, mNm');
subplot(313)
hold on
plot(Time,Torques(3,:,1),'r','LineWidth',1);
plot(Time,Torques(3,:,3),'b','LineWidth',1);
plot([Time(1) Time(end)],[maxTorque maxTorque],'k--','LineWidth',2)
plot([Time(1) Time(end)],[-maxTorque -maxTorque],'k--','LineWidth',2)
axis([Time(1) Time(end) -2*maxTorque 2*maxTorque])
yl3 = ylabel('$\tau_3$, mNm');
xl = xlabel('Time, $t$, s');
set([title1 yl1 yl2 yl3 xl],'interpreter','latex','fontsize',12)
set(leg1,'interpreter','latex','fontsize',8)
saveas(fig11,['mimpcfigures/1normtorques_tf_' num2str(Time(end)) '_noICerr.fig'])

fig12 = figure(12);
subplot(311)
hold on
h1 = plot(Time,Torques(1,:,2),'r','LineWidth',1);
h2 = plot(Time,Torques(1,:,4),'b','LineWidth',1);
h3 = plot([Time(1) Time(end)],[maxTorque maxTorque],'k--','LineWidth',2);
plot([Time(1) Time(end)],[-maxTorque -maxTorque],'k--','LineWidth',2)
axis([Time(1) Time(end) -2*maxTorque 2*maxTorque])
title1 = title('Torque History, 2-Norm');
leg1 = legend([h1 h2 h3],'MI','Mod.','Max. Torque',...
    'Location','Best');
yl1 = ylabel('$\tau_1$, mNm');
subplot(312)
hold on
plot(Time,Torques(2,:,2),'r','LineWidth',1);
plot(Time,Torques(2,:,4),'b','LineWidth',1);
plot([Time(1) Time(end)],[maxTorque maxTorque],'k--','LineWidth',2)
plot([Time(1) Time(end)],[-maxTorque -maxTorque],'k--','LineWidth',2)
axis([Time(1) Time(end) -2*maxTorque 2*maxTorque])
yl2 = ylabel('$\tau_2$, mNm');
subplot(313)
hold on
plot(Time,Torques(3,:,2),'r','LineWidth',1);
plot(Time,Torques(3,:,4),'b','LineWidth',1);
plot([Time(1) Time(end)],[maxTorque maxTorque],'k--','LineWidth',2)
plot([Time(1) Time(end)],[-maxTorque -maxTorque],'k--','LineWidth',2)
axis([Time(1) Time(end) -2*maxTorque 2*maxTorque])
yl3 = ylabel('$\tau_3$, mNm');
xl = xlabel('Time, $t$, s');
set([title1 yl1 yl2 yl3 xl],'interpreter','latex','fontsize',12)
set(leg1,'interpreter','latex','fontsize',8)
saveas(fig12,['mimpcfigures/2normtorques_tf_' num2str(Time(end)) '_noICerr.fig'])

fig13 = figure(13);
hold on
h1 = plot(Time,mTrack(:,1),'r','LineWidth',1);
h2 = plot(Time,mTrack(:,3),'b','LineWidth',1);
h4 = plot(Time(1:length(mOpt)),mOpt,'k','LineWidth',2);
axis tight
title1 = title('Spacecraft Mass History, 1-Norm');
leg1 = legend([h1 h2 h4],...
    ['MI Tracking, $\Delta v = $ ' num2str(deltaVTracking(1)) ' m/s'],...
    ['Mod. Tracking, $\Delta v = $ ' num2str(deltaVTracking(3)) ' m/s'],...
    ['Optimal, $\Delta v = $ ' num2str(deltaVOptimal) ' m/s'],'location',...
    'best');
xl = xlabel('Time, $t$, s');
yl = ylabel('Mass, $m$, kg');
set([leg1 title1 xl yl],'interpreter','latex','fontsize',10)
saveas(fig13,['mimpcfigures/1normmass_tf_' num2str(Time(end)) '_noICerr.fig'])

fig14 = figure(14);
hold on
h1 = plot(Time,mTrack(:,2),'r','LineWidth',1);
h2 = plot(Time,mTrack(:,4),'b','LineWidth',1);
h4 = plot(Time(1:length(mOpt)),mOpt,'k','LineWidth',2);
axis tight
title1 = title('Spacecraft Mass History, 2-Norm');
leg1 = legend([h1 h2 h4],...
    ['MI Tracking, $\Delta v = $ ' num2str(deltaVTracking(2)) ' m/s'],...
    ['Mod. Tracking, $\Delta v = $ ' num2str(deltaVTracking(4)) ' m/s'],...
    ['Optimal, $\Delta v = $ ' num2str(deltaVOptimal) ' m/s'],'location',...
    'best');
xl = xlabel('Time, $t$, s');
yl = ylabel('Mass, $m$, kg');
set([leg1 title1 xl yl],'interpreter','latex','fontsize',10)
saveas(fig14,['mimpcfigures/2normmass_tf_' num2str(Time(end)) '_noICerr.fig'])

end