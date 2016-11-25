function [ A ] = skew( a )
%skew Given a 3x3 skew symmetric matrix, returns the vector a that makes up
%those components.  If given a 1x3 or 3x1 vector a, returns the relevant skew
%symmetric matrix
[m,n] = size(a);
if m==3&&n==1||m==1&&n==3
   A = [0 -a(3) a(2); 
        a(3) 0 -a(1); 
       -a(2) a(1) 0]; 
elseif a+a'==zeros(3)
    A = [a(3,2);
         a(1,3);
         a(2,1)];
end
end

