%Approach Trend
close all, clear all, clc

w1 = 1e-1; %Thrust
w2 = 1;    %Targeting

%fuel parameters
g0 = 9.80665;
I = 800;            %s
umax = 0.26;        %N
ISP = I/(0.5*g0);   %Full tank mass ~0.5kg
mdot = umax/ISP/g0;

%FullSoln
tmax = 60;
fval = 27.8273;
cpuT = 67.7231;

T = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 30, 32, 35, 40, tmax]; %horizon time
f = [0.4986, 0.4986, 0.4988, 0.4988, 0.4989, 0.4990, 0.4989, 0.4990, 0.4990,...
    0.4991, 0.4991, 0.4964, 0.4974, 0.4964, 0.4962, 0.5]; %fuel left
D = [27.9997, 28.0392, 28.0062, 27.9997, 28.0546, 28.0729, 28.1473, 28.0683,...
    28.0895, 27.9294, 27.8358, 21.9689, 21.4243, 27.2636, 27.8093, fval]; %abs val of distance from target
C = [6.9886, 7.5112, 11.89, 13.7647, 19.8051, 42.6595, 129.9068, 210.585, 291.6829,...
    441.3541, 515.9198, 773.6754,774.3038, 1291.5151, 774.3479, cpuT]; %cpu time


%cost function
J = w1*(0.5-f)/mdot+w2*D;

figure
subplot(2,1,1)
hold on
plot(T,J,'bx','linewidth',2)
title('Approach Optimality Trends')
ylabel('Cost Function')

subplot(2,1,2)
plot(T,C,'bx','linewidth',2)
xlabel('Horizon Time [s]')
ylabel('CPU Time [s]')


