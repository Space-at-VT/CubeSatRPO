function [A,B] = getCoupledSystemJacobians(Imat,m,n,rho)

r1x = rho(1);
r1z = rho(2);
r2x = rho(3);
r2y = rho(4);
r3y = rho(5);
r3z = rho(6);
r4y = rho(7);
r4z = rho(8);
r5x = rho(9);
r5y = rho(10);
r6x = rho(11);
r6z = rho(12);

Ahcw = getHCW_Amat(n);
[Aquat,Bquat] = getAmat_Quat(n,Imat);

Bhcw = [zeros(3,6);
        diag(1/m.*ones(3,1)), -diag(1/m.*ones(3,1))];


Bc1 = [0 -r6z r2y;
       r4z 0 -r2x;
       -r4y r6x 0];

Bc2 = [0 -r1z r5y;
       r3z 0 -r5x;
       -r3y r1x 0];


Bcouple = [zeros(4,6);
           Bc1, Bc2];
       

A = [Aquat, zeros(7,6);
     zeros(6,7), Ahcw];
B = [Bquat,Bcouple;
     zeros(6,3),Bhcw];

end

function Ahcw = getHCW_Amat(n)

Ahcw = [0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 1;
        3*n^2 0 0 0 2*n 0;
        0 0 0 -2*n 0 0;
        0 0 -n^2 0 0 0];
end

function [A,B] = getAmat_Quat(n,Imat)

I1 = Imat(1,1);
I2 = Imat(2,2);
I3 = Imat(3,3);

A = [0 0 0 0 0 0 0;
    0 0 0 0 1/2 0 0;
    0 0 0 0 0 1/2 0;
    0 0 0 0 0 0 1/2;
    0 (2*(I2-I3)*n^2/I1) 0 0 0 (I1-I2+I3)*n/I1 0;
    0 0 8*(I1-I3)*n^2/I2 0 -(I2-I1+I3)*n/I2 0 0;
    0 0 0 6*(I1-I2)*n^2/I3 0 0 0];

B = [zeros(4,3); Imat^(-1)];
end