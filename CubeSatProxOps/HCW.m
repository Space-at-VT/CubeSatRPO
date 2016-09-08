function A = HCW(scenario)
n = scenario.n;
A = [zeros(3,3) eye(3)
    3*n^2 0 0 0 2*n 0
    0 0 0 -2*n 0 0
    0 0 -n^2 0 0 0];
end