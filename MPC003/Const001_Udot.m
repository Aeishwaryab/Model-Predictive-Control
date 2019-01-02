%% MPC with constraints on rate of change of the control input. 
% System in defined by a transfer function.
clear all
close all
%% Model 
% Transfer Function. 
% G = 10/(s^2 + 0.1*s + 3);
s = tf('s');
numG = 10;
denG = [1 0.1 3];
% Transfer Function to state space
[Ac, Bc, Cc, Dc] = tf2ss(numG, denG);
% continuous to discrete time
Delta_t = 0.01;
[Am,Bm,Cm,Dm] = c2dm(Ac,Bc,Cc,Dc,Delta_t);
%% Control and prediction horizon
Nc  = 3;
Np  = 20;
rw  = 0.01; % called weighting factor here
rkk = 1;   % set point

%% Augmented system 
[A_e, B_e, C_e, D_e] = ToAugmentedSS(Am, Bm,Cm, Dm);

%% Inequality Constraint 
M = [1 0 0 ;-1 0 0];
gamma = 1*[3;1.5];

%% F and Phi
[F, Phi] = FPhi(A_e, B_e, C_e,D_e,Np,Nc);
R = eye(Nc,Nc)*rw;
Rs = ones(Np,1) * rkk;

E = (Phi'*Phi + R);
Einv = inv(E);
% for Hildert's function 
H = M*Einv *M';


%% From previous code trial 
[n, n_in] = size(B_e);
xm = [1;0];
Xf = zeros(n,1);
N_sim = 100;
r = ones(N_sim,1);
u = 0; % u(k-1) = 0
y = 0;

% Constrained variables
xmc = [1;0];
Xfc = zeros(n,1);
u1 = 0; 
yc = 0;
u_adj = [u1;u1];
for kk = 1:N_sim
    % Hildert's 
    Fi = -2*Phi'*(Rs - F*Xfc); 
    K = gamma + M* Einv *Fi;
    lambda = PrimaDualHildert002(H, K);
    delu_o = -Einv *Fi; % unconstrained control effort.
    delu_op = delu_o - Einv*M'*lambda; % constrained control effort.
    u1 = u1 + delu_op(1,1);
    u_con(kk) = u1;
    y_con(kk) = yc;
    u_adj = [-1*u1;u1];
    xmc_old = xmc;
    xmc = Am*xmc + Bm*u1;
    yc  = Cm*xmc + Dm*u1;
    Xfc = [xmc-xmc_old;yc];
    
    % Unconstrained
    U = Control(Rs,E, F, Phi, Xf);
    deltau = U(1,1);
    u = u + deltau;
    u_free(kk) = u;
    y1(kk) = y ;
    xm_old = xm;
    xm = Am*xm +Bm*u;
    y = Cm*xm+ Dm*u;
    Xf = [xm-xm_old ; y];
    
end
clc
disp("complete")
disp("Plotting...")
k = 0:(N_sim-1);
figure(1)

subplot(211)
hold on
plot(k,y1,'green','LineWidth',2)
plot(k,y_con,'red','LineWidth',1.5,'LineStyle','--')
xlabel('Sampling Instant')
title('Output')
legend('Without Inequality constraints', 'With Inequality constraints')
grid on

subplot(212)
hold on
stairs(k,u_free,'green','LineWidth',2)
stairs(k,u_con,'red','LineWidth',2)
xlabel('Sampling Instant')
title('Control Input')
legend('Without Inequality constraints', 'With Inequality constraints')
hold on
grid on
