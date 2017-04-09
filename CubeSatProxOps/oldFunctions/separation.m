function d = separation(p1,p2,ax)
if nargin < 3 || isempty(ax)
    d = norm(p1-p2,2);
else
    d = abs(p1(ax)-p2(ax));
end