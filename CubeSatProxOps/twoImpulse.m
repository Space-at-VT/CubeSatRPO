function twoImpulse()
clear,clc
close all

r = 6803.1366e3;
n = sqrt(3.986e14/r^3);
TP = 2*pi/n;

t0 = 0;
tf = 1;

tspan = TP*linspace(t0,tf,1000);
% R = linspace(1,20,10);
R = [2,4,8,16];

figure
hold on
for jj = 1:length(R)
    for ii = 1:length(tspan)
        dv(ii) = twoIM(tspan(ii),R(jj));
    end
    dvhalf = twoIM(TP/2,R(jj));
    plot(tspan/TP,dv/dvhalf,'linewidth',1.25)

end

grid on
axis([t0 tf 0 2])
xl = xlabel('Maneuver Time, TP');
yl = ylabel('\Deltav/\Deltav_{0.5}');
lg = legend({'R = 2','R = 4','R = 8','R = 16'});
set([xl,yl,lg],'fontsize',12)


end


function dv = twoIM(t,R)
r = 6803.1366e3;
n = sqrt(3.986e14/r^3);
% t = 2*pi/n*1.1;

x0 = 200;
xf = x0/R;

r0 = [x0,0,0]';
v0 = [0,-2*n*r0(1),0]';

rf = [xf,0,0]';
vf = [0,-2*n*rf(1),0]';

phirr = [4-3*cos(n*t)       0 0
    -6*n*t+6*sin(n*t)  1 0
    0                  0 cos(n*t)];
phirv = [1/n*sin(n*t)       2/n-2/n*cos(n*t)    0
    -2/n+2/n*cos(n*t)  4/n*sin(n*t)-3*t    0
    0                  0                   1/n*sin(n*t)];
v1 = phirv\(rf-phirr*r0);
v1 = norm(v1-v0);

% Plot
X0 = [r0;v0];

Xf = HCWSTM(X0,n,t);
v2 = norm(Xf(4:6)-vf);
dv = v1+v2;

% figure('position',[100 100 1280 720])
% hold on
% plot3(0,0,0,'k+','linewidth',1.25,'Markersize',8);
% plot3(r0(1),r0(2),r0(3),'ko','Markerfacecolor','g','Markersize',8);
% plot3(X(:,1),X(:,2),X(:,3),'linewidth',1.25)
% grid on
% axis('equal','tight')
% camva(8)
% xl = xlabel('Radial, x, m');
% yl = ylabel('In-Track, y, m');
% zl = zlabel('Cross-Track, z, m');
% tt = title('Problem 3');
% lg = legend({'Origin','Initial Position','Rendezvous Maneuver'});
% set([xl,yl,zl,lg,tt],'FontSize',12)
% view([-60,30])
end

function x = HCWSTM(x0,n,t)
ii = 1;
x = [4-3*cos(n*t(ii)) 0 0 1/n*sin(n*t(ii)) 2/n - 2/n*cos(n*t(ii)) 0;
    -6*n*t(ii)+6*sin(n*t(ii)) 1 0 -2/n+2/n*cos(n*t(ii)) 4/n*sin(n*t(ii))-3*t(ii) 0;
    0 0 cos(n*t(ii)) 0 0 1/n*sin(n*t(ii));
    3*n*sin(n*t(ii)) 0 0 cos(n*t(ii)) 2*sin(n*t(ii)) 0;
    -6*n+6*n*cos(n*t(ii)) 0 0 -2*sin(n*t(ii)) -3+4*cos(n*t(ii)) 0;
    0 0 -n*sin(n*t(ii)) 0 0 cos(n*t(ii))]*x0;

end