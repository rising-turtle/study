function [Cost] = computeCost2(x, r, theta)

m = length(r);
delta = exp(-exp(weibull_tmp(x, theta))) - r;
Cost = (delta'*delta)/(2*m);

end