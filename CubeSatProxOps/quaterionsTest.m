function quaterionsTest
close all,
clc

tf = 100;
dt = 1;

w0 = [0.1,0.5,0.1]';
q0 = [0,0,0,1]';
y0 = [w0;q0];
I = 20/12*(0.3^2)*[1,1,1];

tspan = 0:dt:tf;
[t,y] = ode45(@(t,y)quaternions(t,y,I),tspan,y0);

w = y(:,1:3);
q = y(:,4:7);

figure
for ii = 1:size(q,1)
    qi = q(ii,1:3)';
    q4 = q(ii,4);
    qx = [0 -qi(3) qi(2)
          qi(3) 0 -qi(1)
         -qi(2) qi(1) 0];
    R = (q4^2 - qi'*qi)*eye(3)+2*(qi*qi')-2*q4*qx;
    clf
    hold on
    plot3([0,R(2,1)'],[0,R(3,1)'],[0,R(1,1)'],'b','linewidth',2);
    plot3([0,R(2,2)'],[0,R(3,2)'],[0,R(1,2)'],'r','linewidth',2);
    plot3([0,R(2,3)'],[0,R(3,3)'],[0,R(1,3)'],'g','linewidth',2);
    hold off
    grid on
    axis([-3 3 -3 3 -3 3])
    camva(9)
    view(145,15)
    zlabel('Radial, x [m]')
    xlabel('In-track, y [m]')
    ylabel('Cross-track, z [m]')
    pause(1e-2)
end

figure
plot(t,w,'linewidth',2)
grid on
legend({'\omega1','\omega2','\omega3'})
xlabel('Time, t')
ylabel('Angular Velocity, rad/s')

figure
hold on
plot(t,q(:,1),'r','linewidth',2)
plot(t,q(:,2),'y','linewidth',2)
plot(t,q(:,3),'g','linewidth',2)
plot(t,q(:,4),'b','linewidth',2)
hold off
grid on
legend({'q1','q2','q3','q4'})
xlabel('Time, t')
ylabel('Quaternions')

end

function dydt = quaternions(t,y,I)
w = y(1:3);
q = y(4:7);

% if t > 30 && t < 31
%     M = [-0.15,0,0.01]';
% else
    M = [0,0,0]';
% end

dwdt = zeros(3,1);
dwdt(1) = (I(2)-I(3))/I(1)*w(2)*w(3)+M(1)/I(1);
dwdt(2) = (I(3)-I(1))/I(2)*w(1)*w(3)+M(2)/I(2);
dwdt(3) = (I(1)-I(2))/I(3)*w(1)*w(2)+M(3)/I(3);

S = [0 -w(3) w(2)
     w(3) 0 -w(1)
     -w(2) w(1) 0];

dqdt = -1/2*[S;w']*q(1:3)+1/2*q(4)*[eye(3);zeros(1,3)]*w;

dydt = [dwdt;dqdt];

end