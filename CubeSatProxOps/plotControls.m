function plotControls(sat,scenario)
t = 0:scenario.dt:scenario.t;
tf = scenario.t;

figure
subplot(3,3,1)
hold on
stairs(t,sat.ux/sat.umax,'-b','linewidth',2)
plot([0 tf],[0 0],'--k','linewidth',2)
axis([0 tf -1.5 1.5])
grid on
title('Control Signals vs Time')
ylabel('ux')

subplot(3,3,4)
hold on
stairs(t,sat.uy/sat.umax,'-r','linewidth',2)
plot([0 tf],[0 0],'--k','linewidth',2)
axis([0 tf -1.5 1.5])
grid on
ylabel('uy')

subplot(3,3,7)
hold on
stairs(t,sat.uz/sat.umax,'-g','linewidth',2)
plot([0 tf],[0 0],'--k','linewidth',2)
axis([0 tf -1.5 1.5])
grid on
xlabel('Time [s]')
ylabel('uz')

% Velocity
subplot(3,3,2)
plot(t,sat.vx,'-b','linewidth',2)
axis([0 tf 0 1],'auto y')
grid on
ylabel('Vx [m/s]')
title('Velocity vs Time')

subplot(3,3,5)
plot(t,sat.vy,'-r','linewidth',2)
axis([0 tf 0 1],'auto y')
grid on
ylabel('Vy [m/s]')

subplot(3,3,8)
plot(t,sat.vz,'-g','linewidth',2)
axis([0 tf 0 1],'auto y')
grid on
xlabel('Time [s]')
ylabel('Vz [m/s]')

% Position
subplot(3,3,3)
plot(t,sat.x,'-b','linewidth',2)
axis([0 tf 0 1],'auto y')
grid on
ylabel('x [m]')
title('Position vs Time')

subplot(3,3,6)
plot(t,sat.y,'-r','linewidth',2)
axis([0 tf 0 1],'auto y')
grid on
ylabel('y [m]')

subplot(3,3,9)
plot(t,sat.z,'-g','linewidth',2)
axis([0 tf 0 1],'auto y')
grid on
xlabel('Time [s]')
ylabel('z [m]')

figure
subplot(3,1,1)
plot(t,sat.thx*(180/pi),'-b','linewidth',2)
axis([0 tf 0 1],'auto y')
grid on
ylabel('\Theta_1 [deg]')
title('Attitude vs Time')

subplot(3,1,2)
plot(t,sat.thy*(180/pi),'-r','linewidth',2)
axis([0 tf 0 1],'auto y')
grid on
ylabel('\Theta_2 [deg]')

subplot(3,1,3)
plot(t,sat.thz*(180/pi),'-g','linewidth',2)
axis([0 tf 0 1],'auto y')
grid on
ylabel('\Theta_3 [deg]')
xlabel('Time [s]')

end