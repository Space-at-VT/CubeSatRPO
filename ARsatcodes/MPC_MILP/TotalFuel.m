function [fuelCost,dv,mt] = TotalFuel(Time,U,m0,Isp,g0,dt)

fuelCost = 0;
for ii = 1:length(Time)
    fuelCost = fuelCost + norm(U(:,1),1);
end
mt = zeros(1,length(Time));

mt(1) = m0;
for ii = 1:length(Time)-1
    ukn(ii) = norm(U(:,ii));
    mt(ii+1) = mt(ii) + dt*(-ukn(ii)/(Isp*g0));
end
dv = deltaV(m0,Isp,g0,mt(end));
end
function DV = deltaV(m0,g0,Isp,m1)
ve = g0*Isp;

DV = ve.*log(m0./m1);

end

