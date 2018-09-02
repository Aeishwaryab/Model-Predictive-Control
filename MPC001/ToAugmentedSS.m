function [A,B,C] = ToAugmentedSS(Am, Bm, Cm)
% Function transforms the state space model to augmented system model used
% in State Space design of Predictive controllers with Embedded Integrator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[j, n] = size(Am);
[m, l] = size(Cm);
clear j l
A21 = Cm*Am;
A22 = eye(m,1);
A12 = zeros(n,1);
B21 = Cm*Bm;
C11 = zeros(1,n);
C21 = eye(1,m);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A = [Am A12 ; A21 A22];
B = [Bm;B21];
C = [C11,C21];
end

