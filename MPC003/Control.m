function [ U] = Control(Rs,E,F, Phi, delxk )
%% This Function Calculated the optimal Control Input and Cost Function 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n = max(size(E));
rnk = rank(E);
if (n == rnk || det(Phi'*Phi + R))    
    Fi = Phi'*(Rs - F*delxk); 
    U = E\ Fi;
else
    disp("Hessian Matrix not invertible. Cannot compute control input.")
end
end

