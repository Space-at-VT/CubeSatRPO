clear
kalman = kalmanSetup([100;0;0],[0;0;0]);
for t =1:3
[ rEst, vEst, kalman ] = stateEst( [100;0;0], [0;0;0], kalman, t )
end