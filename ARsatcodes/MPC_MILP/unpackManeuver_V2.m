function [Xk,Uk,Uk2,Time,Ts,Nsim,X0,Xf,A,B,Umax,Ub,m0,massHist,Xq,Uq,Isp] = ...
    unpackManeuver_V2(FTS)

Xk = FTS.statestruct.state;
Uk = FTS.statestruct.thruster;
Uk2 = FTS.statestruct.thruster2;
Time = FTS.statestruct.Time;
Ts = FTS.statestruct.Ts;
Nsim = FTS.statestruct.Nsim;
X0 = FTS.statestruct.X0;
Xf = FTS.statestruct.Xf;
A = FTS.statestruct.A;
B = FTS.statestruct.B;
Umax = FTS.statestruct.Umax;
Ub = FTS.statestruct.Ub;
m0 = FTS.statestruct.mass;
massHist = FTS.statestruct.massHist;
Xq = FTS.statestruct.Xq;
Uq = FTS.statestruct.Uq;
Isp = FTS.statestruct.Isp;
end