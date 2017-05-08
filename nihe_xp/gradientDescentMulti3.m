function [theta, J_history] = gradientDescentMulti3(X, y, theta, alpha, num_iters)
%GRADIENTDESCENT Performs gradient descent to learn theta
%   theta = GRADIENTDESENT(X, y, theta, alpha, num_iters) updates theta by 
%   taking num_iters gradient steps with learning rate alpha

% Initialize some useful values
m = length(y); % number of training examples
J_history = zeros(num_iters, 1);

for iter = 1:num_iters

    % ====================== YOUR CODE HERE ======================
    % Instructions: Perform a single gradient step on the parameter vector
    %               theta. 
    %
    % Hint: While debugging, it can be useful to print out the values
    %       of the cost function (computeCost) and gradient here.
    %
    % theta: n+1*1 X: m*(n+1)
    h = weibull_tmp(X, theta);
    diff_theta_1 = theta(2)./(theta(1) - X);
    diff_theta_2 = log(X - theta(1)) - log(theta(3));
    diff_theta_3 = zeros(m, 1) - theta(2)/theta(3);
    
    theta(1) = theta(1) - (alpha/m)*((h-y)'*(diff_theta_1)); 
    theta(2) = theta(2) - (alpha/m)*((h-y)'*(diff_theta_2));
    theta(3) = theta(3) - (alpha/m)*((h-y)'*(diff_theta_3));
    
    
    % theta = theta - (alpha/m) * (X'*(X*theta - y));

    % ============================================================

    % Save the cost J in every iteration    
    J_history(iter) = computeCost(X, y, theta);

end

end