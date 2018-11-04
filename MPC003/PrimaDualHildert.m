function [lambda] = PrimaDualHildert(H, K)
%% Implementaiton of Hilderts quadratic programming 

% initialize lagranges multiplier
n = length (K); % length of K is the no of constraints
lambda = zeros(n,1);  % column vector
tempLam = zeros(n,1);
k = 2;
% loop for all constraints
for j=1:k
for i = 1: n
    if i == 1
        omega = -K(1,1)/H(1,1) - H(1,2:n)*lambda(2:n,1);
        tempLam(1,1) = max(0,omega);
    else
        omega = -K(i,1)/H(i,i) - H(i,i+1:n)*lambda(i+1:n,1) - H(i,1:i-1)*tempLam(1:i-1,1);
        tempLam(i,1) = max(0,omega);
    end
end
diff = tempLam-lambda;
if any(diff)
    k = k+1;
end
lambda = tempLam;
end
end

