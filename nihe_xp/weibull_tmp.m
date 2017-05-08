function [ y ] = weibull_tmp( t, theta )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
y = zeros(length(t), 1);
y = theta(2).*log(t - theta(1)) - theta(2)*log(theta(3));

end

