function [F, Phi] = FPhi(A,B,C,D, Np, Nc)
% Function computes F and Phi matrices used in Cost Function Calculation
% and finding correct control Input
% Np = optimization horizon
% Nc = control horizon
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a = (Np>=Nc);
if(a)    
    F1 = C*A;
    F_final = F1;
    for i = 1: Np-1
        F1 = F1*A;
        F_final = [F_final;F1];
    end
    Phi1 = C*B;
    for i = 1:Np-1
        Phi1 = [Phi1; C*A^(i)*B];
    end
    Phi2 = Phi1;
    len = length(Phi1);
    for i = 1:Nc-1
        Phi_temp = [zeros(i,1);Phi1(1:len-i)];
        Phi2 = [Phi2, Phi_temp];
    end
    Phi3 = C*D;
    for i = 1:Np-1
        Phi3 = [Phi3; C*A^(i)*D];
    end
    Phi4 = [zeros(length(Phi3),1) Phi3];
    for i = 1:Nc-2
        Phi2_temp = [zeros(i,1);Phi3(1:len-i)];
        Phi4 = [Phi4, Phi2_temp];
    end
else
    disp('Np cannot be smaller than Nc')
end   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
F = F_final;
Phi = Phi2+Phi4;
end

