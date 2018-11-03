%% 
% This is a program which minimizes the cost function J with inequality constraints. 
% The decision variable is x 
% Method used: Primal Dual Method with Hilderts Quadratic Programming

%% Cost Junction
% J = 1/2 (x^T E x) + F^T x  ......... cost function where x is the decision variable 
% E = [2 -1 ; -1 1];
% F = [-1;0];
E = [1 0; 0 1];
F = [-2;-2];
%% constraints
% 0<=x1 ; 0<=x2 ; 3 x1 + 2 x2 <= 4
% Written in the form M x <= gamma
% M = [-1 0; 0 -1; 3 2];
% gamma = [0 ; 0 ; 4];
M = [1 0 ; -1 0; 0 1; 0 -1];
gamma = [1 ;0;1;0];
%% global optimal solution 
x_o = -inv(E) *F;

%% H and K 
% Variables needed for Hilderts quadratic programming or even primal dual
% approach

H = M/E *M';
K = gamma + M/E *F;

lambda = PrimaDualHildert(H, K);
x_op = x_o - E\M'*lambda;


