function [Cost] = computeCost(x, y, theta)

m = length(y);
delta = weibull_tmp(x, theta) - y;
Cost = (delta'*delta)/(2*m);

end