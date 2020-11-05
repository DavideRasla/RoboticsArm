function [c, ceq] = nonlin(point)
global obs r p_curr
c(1) = -norm(point - obs) + (r+0.1); 
c(2) = norm(point -obs) - (r+0.6);
c(3) = norm(point - p_curr) - 0.28;
ceq = [];
end