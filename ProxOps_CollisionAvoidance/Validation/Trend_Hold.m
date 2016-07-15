%Hold Trend
close all, clear all, clc

w1 = 1e-1; %Thrust

%fuel parameters
g0 = 9.80665;
I = 800;            %s
umax = 0.26;        %N
ISP = I/(0.5*g0);   %Full tank mass ~0.5kg
mdot = umax/ISP/g0;

%FullSoln
tmax = 60;
fval = 27.8273;
cpuT = 61.7231;

T = [10, 15, 20, 25, 30, 35, 40, 45, 50, tmax]; %horizon time
f = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.4999, 0.4999, 0.5, 0.5-fval*mdot/w1]; %fuel left
C = [5.6418, 5.9697, 7.0277, 8.5848,10.7176,13.2279, 17.9631, 22.7991, 29.7511, cpuT]; %cpu time

%cost function
J = w1*(0.5-f)/mdot;

figure
subplot(2,1,1)
hold on
plot(T,J,'bx','linewidth',2)
title('Hold Optimality Trends')
ylabel('Cost Function')

subplot(2,1,2)
plot(T,C,'bx','linewidth',2)
xlabel('Horizon Time [s]')
ylabel('CPU Time [s]')


