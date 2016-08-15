kalman = RPO_MILP_MPC

figure(1), clf;
subplot(3,1,1);
hold on
plot(kalman.t,kalman.xEst(1,:)-kalman.xAct(1,:))
plot(kalman.t,kalman.xMeas(1,:)-kalman.xAct(1,:), 'r.')
plot(kalman.t,kalman.xEst(1,:)-kalman.xMeas(1,:), 'g.')

subplot(3,1,2);
hold on
plot(kalman.t,kalman.xEst(2,:)-kalman.xAct(2,:))
plot(kalman.t,kalman.xMeas(2,:)-kalman.xAct(2,:), 'r.')
plot(kalman.t,kalman.xEst(2,:)-kalman.xMeas(2,:), 'g.')

subplot(3,1,3);
hold on
plot(kalman.t,kalman.xEst(3,:)-kalman.xAct(3,:))
plot(kalman.t,kalman.xMeas(3,:)-kalman.xAct(3,:), 'r.')
plot(kalman.t,kalman.xEst(3,:)-kalman.xMeas(3,:), 'g.')

figure(2), clf;
hold on
plot(kalman.t,kalman.xEst(4,:)-kalman.xAct(4,:),'r.')
plot(kalman.t,kalman.xEst(5,:)-kalman.xAct(5,:),'go')
plot(kalman.t,kalman.xEst(6,:)-kalman.xAct(6,:),'b*')

figure(3), clf;
hold on;
plot(kalman.t,kalman.u(1,:),'r.');
plot(kalman.t,kalman.u(2,:),'go');
plot(kalman.t,kalman.u(3,:),'b*');