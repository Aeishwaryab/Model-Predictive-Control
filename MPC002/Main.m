%% Tutorial 1.3 Implementation of Receding Horizon Control 
% Plant model is given
% x[k+1] = Ap x[k] + Bp u[k]
% y[k]   = Cp x[k]
clear all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ap = [ 1 1; 0 1];
Bp = [0.5;1];
Cp = [1 0];
Dp = 0.5;
Np = 4; % Prediction horizon
Nc = 2; % Control horizon
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Augmented System Model
% [delx[k+1] ; y[k+1] ] = A_e delx[k] + B_e delu[k]
% y[k] = C_e [delx[k+1] ; y[k+1] ]
[A_e, B_e, C_e, D_e] = ToAugmentedSS(Ap, Bp,Cp, Dp);
%%%%%%%%%%%%%%%%%%%%%%%%%
%% Computing F and Phi matrices 
[F, Phi] = FPhi(A_e, B_e, C_e,D_e,Np,Nc);
[n, n_in] = size(B_e);
xm = [0;0];
Xf = zeros(n,1);
N_sim = 100;
r = ones(N_sim,1);
u = 0; % u(k-1) = 0
y = 0;
rw = 0.05; % called weighting factor here
rkk = 10; % set point

for kk = 1:N_sim
    [J, U] = CostAndControl(rkk,rw, F, Phi, Xf, Np,Nc );
    deltau = U(1,1);
    u = u + deltau;
    u1(kk) = u;
    y1(kk) = y ;
    xm_old = xm;
    xm = Ap*xm +Bp*u;
    y = Cp*xm+Dp*u;
    Xf = [xm-xm_old ; y];
end
clc
disp("complete")
disp("Plotting...")
k = 0:(N_sim-1);
figure(1)
subplot(211)
plot(k,y1)
xlabel('Sampling Instant')
legend('Output')
grid on
hold on
subplot(212)
plot(k,u1)
xlabel('Sampling Instant')
legend('Control')
hold on
grid on
