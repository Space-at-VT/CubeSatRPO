function DV = deltaV(m0,g0,Isp,m1)
ve = g0*Isp;

DV = ve.*log(m0./m1);

end