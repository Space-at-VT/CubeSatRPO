function X = hcwSTM(X0,n,dt)
STM = [4-3*cos(n*dt) 0 0 1/n*sin(n*dt) 2/n - 2/n*cos(n*dt) 0;
        -6*n*dt+6*sin(n*dt) 1 0 -2/n+2/n*cos(n*dt) 4/n*sin(n*dt)-3*dt 0;
        0 0 cos(n*dt) 0 0 1/n*sin(n*dt);
        3*n*sin(n*dt) 0 0 cos(n*dt) 2*sin(n*dt) 0;
        -6*n+6*n*cos(n*dt) 0 0 -2*sin(n*dt) -3+4*cos(n*dt) 0;
        0 0 -n*sin(n*dt) 0 0 cos(n*dt)];
    
X = STM*X0;
end