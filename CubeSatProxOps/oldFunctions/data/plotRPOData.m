clear,clc
close all
% 
% load('Point')
% load('NoPoint')

% x = 5:15;
% figure
% hold on
% p1 = plot(x,fuel*1e3,'ob');
% b1 = polyfit(x,fuel*1e3,1);
% l1 = plot(x,polyval(b1,x),'-b');
% 
% p2 = plot(x,fuelPoint*1e3,'or');
% b2 = polyfit(x,fuelPoint*1e3,1);
% l2 = plot(x,polyval(b2,x),'-r');
% grid on
% xlabel('Radial Distance Allowed, m')
% ylabel('Fuel Used, g')
% legend([p1,p2],{'No Attitude Pointing','Targeted Attitude Pointing'},...
%     'location','best')


load('data12_3_1')
load('data12_3_2')
V = (1:25);

figure
hold on
p1 = plot(V,fuel*1e3,'ob');
p2 = plot(V,fuelp*1e3,'rx');

grid on
xlabel('Constrained Volume, m^3')
ylabel('Fuel Used, g')
legend([p1,p2],{'No Attitude Pointing','Targeted Attitude Pointing'},...
    'location','best')