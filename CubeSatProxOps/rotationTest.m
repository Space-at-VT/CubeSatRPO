clear,clc
close all

up = [1,0,0]';

th1 = 90*pi/180;
th2 = 90*pi/180;
th3 = 0*pi/180;

R = rot(th3,3)*rot(th2,1)*rot(th1,3)
u = R\up

