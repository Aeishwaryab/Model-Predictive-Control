function [J, U] = CostAndControl(rk,rw, F, Phi, delxk, Np,Nc )
%% This Function Calculated the optimal Control Input and Cost Function 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% rk = Set point Rs = [1 1 .... 1]rk
% xk = Initial state
% rw = weight on control input
Rs = ones(Np,1) * rk;
R = eye(Nc,Nc)*rw;
Term0 = Phi'*Phi + R;
n = max(size(Term0));
rnk = rank(Term0);

if (n == rnk || det(Phi'*Phi + R))
    disp("Hessian Matrix is invertible")
    Term1 = inv(Term0);    
    Term2 = Rs - F*delxk; 
    U = Term1 * Phi' * Term2;
    J = Term2'*Term2 - 2*U'*Phi'*Term2 + U'*Term0*U;
else
    disp("Hessian Matrix not invertible. Cannot compute control input.")
end
end

